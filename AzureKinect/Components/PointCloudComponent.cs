using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Grasshopper.Kernel;
using K4AdotNet.Sensor;
using Rhino.Geometry;

namespace AzureKinect.Components
{
    public class PointCloudComponent : GH_Component
    {
        // 式式式 Reusable resources (allocated once, kept alive across solves) 式式
        // This is the whole performance story: avoid per-frame allocation.

        private short[] _xyzBuffer;
        private Image _xyzImage;
        private Transformation _transformation;
        private int _imageWidth;
        private int _imageHeight;

        // 式式式 Constructor 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式
        public PointCloudComponent()
          : base("Point Cloud", "PtCloud",
                 "Streams point cloud data from a Kinect device. Outputs a single PointCloud object per frame (not a list of points).",
                 "Appendage", "Kinect")
        {
        }

        // 式式式 Inputs 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Kinect", "Dev",
                "Live Kinect device handle from Kinect Device component.",
                GH_ParamAccess.item);

            pManager.AddBooleanParameter("Active", "On",
                "Activate point cloud streaming.",
                GH_ParamAccess.item, false);

            pManager.AddIntegerParameter("Stride", "Stride",
                "Downsample: 1 = every pixel, 2 = every 2nd, 4 = every 4th. Higher = faster + sparser.",
                GH_ParamAccess.item, 1);
        }

        // 式式式 Outputs 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Point Cloud", "PtCloud",
                "Point cloud (in mm, floor-corrected). Valid points only ? for visualization.",
                GH_ParamAccess.item);

            pManager.AddGenericParameter("Grid", "Grid",
            "Structured Point3d grid (with Unset for invalid pixels). Use for mesh construction.",
            GH_ParamAccess.item);

        }

        // 式式式 Main solver 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_KinectDevice deviceGoo = null;
            bool active = false;
            int stride = 1;

            DA.GetData(0, ref deviceGoo);
            DA.GetData(1, ref active);
            DA.GetData(2, ref stride);

            // No device or not active ⊥ release any buffers we hold, exit
            if (deviceGoo == null || !deviceGoo.IsValid || !active)
            {
                if (_transformation != null) ReleaseBuffers();
                return;
            }

            stride = Math.Max(1, stride);

            // Lazy buffer allocation on first activation
            // (also reallocates if depth mode changed)
            var cal = deviceGoo.Calibration;
            int width = cal.DepthCameraCalibration.ResolutionWidth;
            int height = cal.DepthCameraCalibration.ResolutionHeight;

            if (_transformation == null || _imageWidth != width || _imageHeight != height)
            {
                ReleaseBuffers();
                try
                {
                    _imageWidth = width;
                    _imageHeight = height;

                    int pixelCount = width * height;
                    _xyzBuffer = new short[pixelCount * 3];

                    int rowStrideBytes = width * sizeof(short) * 3;
                    _xyzImage = Image.CreateFromArray(
                        _xyzBuffer,
                        ImageFormat.Custom,
                        width, height, rowStrideBytes);

                    _transformation = cal.CreateTransformation();
                }
                catch (Exception ex)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                        $"Buffer init failed: {ex.Message}");
                    ReleaseBuffers();
                    return;
                }
            }

            // Capture a frame, project depth ⊥ xyz into reused buffer
            try
            {
                using (var capture = deviceGoo.AcquireCapture?.Invoke())
                {
                    if (capture == null) return;
                    var depthImage = capture.DepthImage;
                    if (depthImage == null) return;

                    _transformation.DepthImageToPointCloud(
                        depthImage,
                        CalibrationGeometry.Depth,
                        _xyzImage);
                }
            }
            catch (Exception ex)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                    $"Capture failed: {ex.Message}");
                return;
            }

            // Convert xyz buffer ⊥ Rhino PointCloud (with stride + floor transform)
            var (cloud, grid, gridWidth, gridHeight) = BuildPointCloud(stride, deviceGoo.FloorTransform);

            var gridGoo = new GH_KinectPointGrid(grid, gridWidth, gridHeight);

            DA.SetData(0, cloud);
            DA.SetData(1, gridGoo);
        }

        private (PointCloud cloud, Point3d[] grid, int gridWidth, int gridHeight) BuildPointCloud(
    int stride, Transform floorTransform)
        {
            int rowCount = (_imageHeight + stride - 1) / stride;
            int colCount = (_imageWidth + stride - 1) / stride;
            int gridSize = rowCount * colCount;

            var grid = new Point3d[gridSize];

            // Parallel: fill grid with valid points or Point3d.Unset for invalid pixels.
            Parallel.For(0, rowCount, rowIdx =>
            {
                int row = rowIdx * stride;
                int rowOffset = row * _imageWidth * 3;
                int rowBaseWriteIdx = rowIdx * colCount;

                for (int colIdx = 0; colIdx < colCount; colIdx++)
                {
                    int col = colIdx * stride;
                    int i = rowOffset + col * 3;
                    short x = _xyzBuffer[i];
                    short y = _xyzBuffer[i + 1];
                    short z = _xyzBuffer[i + 2];

                    int writeIdx = rowBaseWriteIdx + colIdx;

                    if (x == 0 && y == 0 && z == 0)
                    {
                        grid[writeIdx] = Point3d.Unset;
                    }
                    else
                    {
                        var p = new Point3d(x, z, -y);
                        p.Transform(floorTransform);
                        grid[writeIdx] = p;
                    }
                }
            });

            // Sequential filter: collect valid points for the visualization PointCloud.
            // Fast linear pass ? ~5ms for 1M points.
            var validPoints = new List<Point3d>(gridSize);
            for (int i = 0; i < gridSize; i++)
            {
                if (grid[i].IsValid) validPoints.Add(grid[i]);
            }

            var cloud = new PointCloud();
            cloud.AddRange(validPoints);

            return (cloud, grid, colCount, rowCount);
        }

        // 式式式 Buffer lifecycle 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式
        private void ReleaseBuffers()
        {
            try { _transformation?.Dispose(); } catch { }
            try { _xyzImage?.Dispose(); } catch { }
            _transformation = null;
            _xyzImage = null;
            _xyzBuffer = null;
            _imageWidth = 0;
            _imageHeight = 0;
        }

        public override void RemovedFromDocument(GH_Document document)
        {
            base.RemovedFromDocument(document);
            ReleaseBuffers();
        }

        // 式式式 Boilerplate 式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式式
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        protected override System.Drawing.Bitmap Icon => IconLoader.Load("DepthCamera24.png");

        public override Guid ComponentGuid =>
            new Guid("b7d2e9f4-3a5c-4e8b-9d1f-7c4a8e2b6d5a");
    }
}





