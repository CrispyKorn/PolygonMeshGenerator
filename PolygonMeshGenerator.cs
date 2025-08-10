using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Linq;
using System;

public class PolygonMeshGenerator : MonoBehaviour
{
    private PolygonCollider2D _polygonCollider;
    private MeshFilter _meshFilter2D;

    private LinkedList<int> _polygon;
    private List<int> _convexVertices;
    private List<int> _reflexVertices;
    private List<int> _earTips;

    private struct Triangle
    {
        public Vector2 vl, v, vr;

        public Triangle(Vector2 vl, Vector2 v, Vector2 vr)
        {
            this.vl = vl;
            this.v = v;
            this.vr = vr;
        }
    }

    public void UpdateMesh()
    {
        _polygonCollider = GetComponent<PolygonCollider2D>();
        _meshFilter2D = GetComponent<MeshFilter>();
        _particleSys = GetComponentInChildren<ParticleSystem>();

        Mesh mesh = new Mesh();

        Vector2[] points = _polygonCollider.points;

        // Triangulate by ear clipping
        int[] triangles = CalculateTriangles(points);
        // Reverse points to order triangles clockwise 
        // (Unity meshes use clockwise winding order while polygon colliders use counterclockwise winding order)
        Array.Reverse(triangles); 

        // Convert Vector2 points to Vector3 vertices
        Vector3[] vertices = new Vector3[points.Length];
        Vector2[] uvs = new Vector2[points.Length];
        for (int i = 0; i < points.Length; i++)
        {
            vertices[i] = points[i];
            uvs[i] = points[i];
        }

        // Create mesh and apply
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.uv = uvs;
        mesh.RecalculateNormals();

        if (_meshFilter2D != null) _meshFilter2D.mesh = mesh;
    }

    /// <summary>
    /// Triangulates a given set of vertices by ear clipping.
    /// <see href="https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf">Source Paper</see>
    /// </summary>
    /// <param name="vertices">The polygon's vertices.</param>
    /// <returns>The array of triangles.</returns>
    private int[] CalculateTriangles(Vector2[] vertices)
    {
        List<int> triangles = new();

        _polygon = new();
        _convexVertices = new();
        _reflexVertices = new();
        _earTips = new();

        // Populate polygon
        for (int i = 0; i < vertices.Length; i++)
        {
            _polygon.AddLast(i);
        }

        // Find convex and reflex vertices
        int node = _polygon.First;
        while (node != null)
        {
            int vl = GetPrevious(node, _polygon.Last).Value;
            int v = node.Value;
            int vr = GetNext(node, _polygon.First).Value;
            Triangle tri = new Triangle(vertices[vl], vertices[v], vertices[vr]);

            if (GetIsConvex(tri)) _convexVertices.Add(v);
            else _reflexVertices.Add(v);

            node = node.Next;
        }

        // Find and add ear tips
        node = _polygon.First;
        while (node != null)
        {
            if (_convexVertices.Contains(node.Value))
            {
                int vl = GetPrevious(node, _polygon.Last).Value;
                int v = node.Value;
                int vr = GetNext(node, _polygon.First).Value;
                Triangle tri = new Triangle(vertices[vl], vertices[v], vertices[vr]);

                if (GetIsEar(tri, vertices)) _earTips.Add(v);
            }

            node = node.Next;
        }

        // Begin ear removal
        while (_polygon.Count > 3)
        {
            // Add ear triangle to triangles
            int ear = _earTips.First();

            int currentNode = _polygon.Find(ear);
            int vl = GetPrevious(currentNode, _polygon.Last).Value;
            int v = currentNode.Value;
            int vr = GetNext(currentNode, _polygon.First).Value;

            triangles.Add(vl);
            triangles.Add(v);
            triangles.Add(vr);

            _earTips.RemoveAt(0);
            _polygon.Remove(ear);

            // Re-evaluate adjacent vertices
            int vlp = GetPrevious(_polygon.Find(vl), _polygon.Last).Value;
            int vrn = GetNext(_polygon.Find(vr), _polygon.First).Value;

            EvaluateTriangle(vlp, vl, vr, vertices); // Reevaluate leftward adjacent vertex
            EvaluateTriangle(vl, vr, vrn, vertices); // Reevaluate rightward adjacent vertex
        }

        triangles.Add(_polygon.First.Value);
        triangles.Add(_polygon.First.Next.Value);
        triangles.Add(_polygon.First.Next.Next.Value);

        return triangles.ToArray();
    }

    /// <summary>
    /// Gets the next node in the list, cyclically.
    /// </summary>
    /// <param name="node">The node from which to get the next node.</param>
    /// <param name="first">The first item in the list.</param>
    private LinkedListNode<int> GetNext(LinkedListNode<int> node, LinkedListNode<int> first)
    {
        if (node.Next == null) return first;
        else return node.Next;
    }

    /// <summary>
    /// Gets the previous node in the list, cyclically.
    /// </summary>
    /// <param name="node">The node from which to get the previous node.</param>
    /// <param name="first">The last item in the list.</param>
    private LinkedListNode<int> GetPrevious(LinkedListNode<int> node, LinkedListNode<int> last)
    {
        if (node.Previous == null) return last;
        else return node.Previous;
    }

    /// <summary>
    /// Checks if three vertices' edges create a convex or reflex angle (relative to their left side)
    /// </summary>
    /// <param name="tri">The triangle representing the three vertices.</param>
    /// <returns>Whether the angle is convex.</returns>
    private bool GetIsConvex(Triangle tri)
    {
        Vector2 lhs = tri.v - tri.vl;
        Vector2 rhs = tri.vr - tri.v;
        float determinate = lhs.x * rhs.y - lhs.y * rhs.x;
        
        return determinate > 0f;
    }

    /// <summary>
    /// Checks if the given point is within the given triangle.
    /// </summary>
    /// <param name="tri">The triangle to check for a contained point.</param>
    /// <param name="p">The point to check for.</param>
    /// <returns>Whether the triangle contains the point.</returns>
    private bool IsInsideTriangle(Triangle tri, Vector2 p)
    {
        float vlp_x = p.x - tri.vl.x;
        float vlp_y = p.y - tri.vl.y;
        float vp_x = p.x - tri.v.x;
        float vp_y = p.y - tri.v.y;

        // Check if point is to the left of line {vl, v}
        float vlv_p = (tri.v.x - tri.vl.x) * vlp_y - (tri.v.y - tri.vl.y) * vlp_x;
        bool leftof_vlv = vlv_p > 0f;

        // Check if point is to the left of line {vl, vr}
        float vlvr_p = (tri.vr.x - tri.vl.x) * vlp_y - (tri.vr.y - tri.vl.y) * vlp_x;
        bool leftof_vlvr = vlvr_p > 0f;

        // Check if point is to the left of line {v, vr}
        float vvr_p = (tri.vr.x - tri.v.x) * vp_y - (tri.vr.y - tri.v.y) * vp_x;
        bool leftof_vvr = vvr_p > 0f;

        if (leftof_vlvr == leftof_vlv || leftof_vvr != leftof_vlv) return false;
        return true;
    }

    /// <summary>
    /// Checks if a given triangle is an ear.
    /// </summary>
    /// <param name="tri">The triangle to check.</param>
    /// <param name="vertices">The polygon's vertices.</param>
    /// <returns>Whether the triangle is an ear.</returns>
    private bool GetIsEar(Triangle tri, Vector2[] vertices)
    {
        //If there are no reflex vertices inside this tri, we have an ear
        for (int i = 0; i < _reflexVertices.Count; i++)
        {
            Vector2 reflexPos = vertices[_reflexVertices[i]];
            if (reflexPos == tri.vl || reflexPos == tri.v || reflexPos == tri.vr) continue;

            if (IsInsideTriangle(tri, reflexPos)) return false;
        }

        return true;
    }

    /// <summary>
    /// Evaluates a given vertex's properties.
    /// </summary>
    /// <param name="vl">The vertex to the left of the vertex to evaluate.</param>
    /// <param name="v">The vertex to evaluate.</param>
    /// <param name="vr">The vertex to the right of the vertex to evaluate.</param>
    /// <param name="vertices">The polygon's vertices.</param>
    private void EvaluateTriangle(int vl, int v, int vr, Vector2[] vertices)
    {
        Triangle tri = new Triangle(vertices[vl], vertices[v], vertices[vr]);

        if (_reflexVertices.Contains(v))
        {
            //Check if vertex has become convex
            if (GetIsConvex(tri))
            {
                _reflexVertices.Remove(v);
                _convexVertices.Add(v);
            }
        }

        //Check if convex vertex is ear
        if (_convexVertices.Contains(v))
        {
            if (GetIsEar(tri, vertices))
            {
                if (!_earTips.Contains(v)) _earTips.Add(v);
            }
            else _earTips.Remove(v);
        }
    }
}