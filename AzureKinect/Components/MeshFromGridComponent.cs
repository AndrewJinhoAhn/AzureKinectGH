using System;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace AzureKinect.Components
{
    public class MeshFromGridComponent : GH_Component
    {
        public MeshFromGridComponent()
          : base("Mesh from Grid", "GridMesh",
                 "Builds a quad mesh from a structured point grid (from Point Cloud component). " +
                 "Culls faces whose edges exceed Max Edge — useful for separating disconnected surfaces.",
                 "Azure Kinect", "Util")
        {
        }

        // ─── Inputs ─────────────────────────────────────────────────────
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Grid", "Grid",
                "Structured Point3d grid from Point Cloud component.",
                GH_ParamAccess.item);

            pManager.AddNumberParameter("Max Edge", "MaxEdge",
                "Maximum edge length (mm). Quads with any edge longer than this are skipped. " +
                "Use to separate person from background (e.g., 200 mm).",
                GH_ParamAccess.item, 200.0);
        }

        // ─── Outputs ────────────────────────────────────────────────────
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M",
                "Quad mesh constructed from the grid.",
                GH_ParamAccess.item);
        }

        // ─── Main solver ────────────────────────────────────────────────
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_KinectPointGrid gridGoo = null;
            double maxEdge = 200.0;

            if (!DA.GetData(0, ref gridGoo) || gridGoo == null || !gridGoo.IsValid)
                return;

            DA.GetData(1, ref maxEdge);

            Point3d[] points = gridGoo.Value;
            int width = gridGoo.Width;
            int height = gridGoo.Height;
            double maxEdgeSq = maxEdge * maxEdge;   // squared, avoids sqrt in inner loop

            var mesh = new Mesh();

            // ── Vertices: add all grid slots (Unset included).
            // We keep 1:1 mapping between grid index and vertex index here so
            // face index math is straightforward. CullUnused will remove the
            // Unset / orphan vertices at the end.
            for (int i = 0; i < points.Length; i++)
                mesh.Vertices.Add(points[i]);

            // ── Faces: for each grid cell, build a quad if all corners are
            // valid AND all 4 edges are shorter than maxEdge.
            for (int row = 0; row < height - 1; row++)
            {
                int rowBase = row * width;
                int nextRowBase = (row + 1) * width;

                for (int col = 0; col < width - 1; col++)
                {
                    int a = rowBase + col;         // upper-left
                    int b = rowBase + col + 1;     // upper-right
                    int c = nextRowBase + col + 1; // lower-right
                    int d = nextRowBase + col;     // lower-left

                    var pa = points[a];
                    var pb = points[b];
                    var pc = points[c];
                    var pd = points[d];

                    // Skip if any corner is Unset
                    if (!pa.IsValid || !pb.IsValid || !pc.IsValid || !pd.IsValid)
                        continue;

                    // Skip if any edge exceeds maxEdge
                    if (pa.DistanceToSquared(pb) > maxEdgeSq) continue;
                    if (pb.DistanceToSquared(pc) > maxEdgeSq) continue;
                    if (pc.DistanceToSquared(pd) > maxEdgeSq) continue;
                    if (pd.DistanceToSquared(pa) > maxEdgeSq) continue;

                    mesh.Faces.AddFace(a, b, c, d);
                }
            }

            // ── Cleanup
            mesh.Vertices.CullUnused();   // Removes Unset and orphan vertices
            mesh.Vertices.CombineIdentical(true, true);
            mesh.Faces.CullDegenerateFaces();
            mesh.Compact();                // Rebuilds internal indices
            mesh.Normals.ComputeNormals(); // Vertex normals for shading

            DA.SetData(0, mesh);
        }

        // ─── Boilerplate ────────────────────────────────────────────────
        protected override System.Drawing.Bitmap Icon => IconLoader.Load("mesh_icon.png");

        public override Guid ComponentGuid =>
            new Guid("d8a5f2c1-9b4e-4f7a-8c3d-2e6a9b5d1f7c");
    }
}