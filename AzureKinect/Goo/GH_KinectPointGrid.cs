using System;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace AzureKinect
{
    /// <summary>
    /// Grasshopper goo wrapper around a structured grid of Point3d values.
    /// Layout: Value[row * Width + col] is the 3D point for pixel (row, col).
    /// Invalid pixels are stored as Point3d.Unset.
    ///
    /// Used to pass depth-image-aligned point grids from PointCloud component
    /// to downstream components like MeshFromGrid.
    /// </summary>
    public class GH_KinectPointGrid : GH_Goo<Point3d[]>
    {
        public int Width { get; set; }
        public int Height { get; set; }

        public GH_KinectPointGrid() : base() { }

        public GH_KinectPointGrid(Point3d[] points, int width, int height)
            : base(points)
        {
            Width = width;
            Height = height;
        }

        public override bool IsValid =>
            Value != null
            && Width > 0
            && Height > 0
            && Value.Length == Width * Height;

        public override string TypeName => "Kinect Point Grid";

        public override string TypeDescription =>
            "Structured Point3d grid (Width × Height) with Unset for invalid pixels.";

        public override IGH_Goo Duplicate()
        {
            // Share the same underlying array — content is treated as immutable per frame.
            return new GH_KinectPointGrid
            {
                Value = this.Value,
                Width = this.Width,
                Height = this.Height
            };
        }

        public override string ToString()
        {
            if (Value == null) return "Kinect Point Grid (null)";
            return $"Kinect Point Grid ({Width} × {Height}, {Value.Length} slots)";
        }
    }
}