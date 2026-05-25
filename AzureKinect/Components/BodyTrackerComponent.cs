using System;
using System.Collections.Generic;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using K4AdotNet.BodyTracking;
using K4AdotNet.Sensor;
using Rhino.Geometry;

namespace AzureKinect.Components
{
    public class BodyTrackerComponent : GH_Component
    {
        // ─── State ──────────────────────────────────────────────────────
        // The tracker is expensive to create (loads ONNX models, allocates
        // GPU memory). Keep it alive across solves once Active goes ON.
        private Tracker _tracker;

        // ─── Bone hierarchy (parent → child) for the 32-joint skeleton ──
        // Used to draw connection lines between joints.
        private static readonly (JointType from, JointType to)[] Bones = new[]
        {
            (JointType.SpineNavel,    JointType.Pelvis),
            (JointType.SpineChest,    JointType.SpineNavel),
            (JointType.Neck,          JointType.SpineChest),
            (JointType.ClavicleLeft,  JointType.SpineChest),
            (JointType.ShoulderLeft,  JointType.ClavicleLeft),
            (JointType.ElbowLeft,     JointType.ShoulderLeft),
            (JointType.WristLeft,     JointType.ElbowLeft),
            (JointType.HandLeft,      JointType.WristLeft),
            (JointType.HandTipLeft,   JointType.HandLeft),
            (JointType.ThumbLeft,     JointType.WristLeft),
            (JointType.ClavicleRight, JointType.SpineChest),
            (JointType.ShoulderRight, JointType.ClavicleRight),
            (JointType.ElbowRight,    JointType.ShoulderRight),
            (JointType.WristRight,    JointType.ElbowRight),
            (JointType.HandRight,     JointType.WristRight),
            (JointType.HandTipRight,  JointType.HandRight),
            (JointType.ThumbRight,    JointType.WristRight),
            (JointType.HipLeft,       JointType.Pelvis),
            (JointType.KneeLeft,      JointType.HipLeft),
            (JointType.AnkleLeft,     JointType.KneeLeft),
            (JointType.FootLeft,      JointType.AnkleLeft),
            (JointType.HipRight,      JointType.Pelvis),
            (JointType.KneeRight,     JointType.HipRight),
            (JointType.AnkleRight,    JointType.KneeRight),
            (JointType.FootRight,     JointType.AnkleRight),
            (JointType.Head,          JointType.Neck),
            (JointType.Nose,          JointType.Head),
            (JointType.EyeLeft,       JointType.Head),
            (JointType.EarLeft,       JointType.Head),
            (JointType.EyeRight,      JointType.Head),
            (JointType.EarRight,      JointType.Head),
        };

        // ─── Constructor ────────────────────────────────────────────────
        public BodyTrackerComponent()
          : base("Body Tracker", "Body",
                 "Tracks human bodies in a Kinect feed. Emits per-body skeleton points, bone lines, and persistent body IDs.",
                 "Azure Kinect", "Device")
        {
        }

        // ─── Inputs ─────────────────────────────────────────────────────
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Device", "Dev",
                "Live Kinect device handle from Kinect Device component.",
                GH_ParamAccess.item);

            pManager.AddBooleanParameter("Active", "On",
                "Activate body tracking. First activation takes 5–10s (model load).",
                GH_ParamAccess.item, false);

            pManager.AddIntegerParameter("Min Confidence", "Conf",
                "Minimum joint confidence: 0=None (keep all), 1=Low, 2=Medium, 3=High. Below this, joints output as Point3d.Unset.",
                GH_ParamAccess.item, 1);
        }

        // ─── Outputs ────────────────────────────────────────────────────
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Skeleton Points", "Pts",
                "Per-body skeleton joint positions. Each branch = one body, 32 joints per branch.",
                GH_ParamAccess.tree);

            pManager.AddLineParameter("Skeleton Connections", "Conn",
                "Per-body bone connections. Branches align with Skeleton Points.",
                GH_ParamAccess.tree);

            pManager.AddIntegerParameter("Body IDs", "IDs",
                "Persistent body IDs assigned by K4ABT. Index aligns with tree branches.",
                GH_ParamAccess.list);
        }

        // ─── Main solver ────────────────────────────────────────────────
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_KinectDevice deviceGoo = null;
            bool active = false;
            int confThreshold = 1;

            DA.GetData(0, ref deviceGoo);
            DA.GetData(1, ref active);
            DA.GetData(2, ref confThreshold);

            // No valid device or not active → ensure tracker is stopped, exit
            if (deviceGoo == null || !deviceGoo.IsValid || !active)
            {
                if (_tracker != null) StopTracker();
                return;
            }

            // Lazy-create the tracker on first activation
            if (_tracker == null)
            {
                try
                {
                    var modelPath = System.IO.Path.Combine(
                        AzureKinectInfo.PluginDirectory ?? "",
                        "dnn_model_2_0_op11.onnx");

                    var config = TrackerConfiguration.Default;
                    config.ProcessingMode = TrackerProcessingMode.GpuDirectML;
                    config.ModelPath = modelPath;
                    _tracker = new Tracker(deviceGoo.Calibration, config);
                }
                catch (Exception ex)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                        $"Failed to create body tracker: {ex.Message}");
                    return;
                }
            }

            // Capture a frame and feed it into the tracker
            try
            {
                using (var capture = deviceGoo.AcquireCapture?.Invoke())
                {
                    if (capture == null) return;
                    _tracker.TryEnqueueCapture(capture);
                }
            }
            catch (Exception ex)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                    $"Capture failed: {ex.Message}");
                return;
            }

            // Try to pop a result (may not be ready every frame)
            var skeletonTree = new DataTree<Point3d>();
            var connectionTree = new DataTree<Line>();
            var bodyIds = new List<int>();

            if (_tracker.TryPopResult(out var bodyFrame))
            {
                using (bodyFrame)
                {
                    int count = bodyFrame.BodyCount;
                    for (int i = 0; i < count; i++)
                    {
                        bodyFrame.GetBodySkeleton(i, out var skeleton);
                        int bodyId = (int)bodyFrame.GetBodyId(i);
                        var path = new GH_Path(i);

                        // Extract all 32 joints
                        var joints = new Point3d[32];
                        for (int j = 0; j < 32; j++)
                        {
                            joints[j] = JointToPoint3d(
                                skeleton[(JointType)j],
                                confThreshold,
                                deviceGoo.FloorTransform);
                        }
                        skeletonTree.AddRange(joints, path);

                        // Build bone connection lines
                        foreach (var (from, to) in Bones)
                        {
                            var p1 = joints[(int)from];
                            var p2 = joints[(int)to];
                            connectionTree.Add(
                                (p1.IsValid && p2.IsValid) ? new Line(p1, p2) : Line.Unset,
                                path);
                        }

                        bodyIds.Add(bodyId);
                    }
                }
            }

            DA.SetDataTree(0, skeletonTree);
            DA.SetDataTree(1, connectionTree);
            DA.SetDataList(2, bodyIds);
        }

        // ─── Helpers ────────────────────────────────────────────────────

        /// <summary>
        /// Converts a K4ABT joint to a Rhino Point3d:
        ///  - Filters by confidence threshold (returns Unset if below).
        ///  - Applies K4A→Rhino coordinate swap (X, Y, Z) → (X, Z, -Y).
        ///  - Applies the floor correction transform from the device handle.
        /// </summary>
        private static Point3d JointToPoint3d(Joint joint, int confThreshold, Transform floorTransform)
        {
            if ((int)joint.ConfidenceLevel < confThreshold)
                return Point3d.Unset;

            float x = joint.PositionMm.X;
            float y = joint.PositionMm.Y;
            float z = joint.PositionMm.Z;

            var p = new Point3d(x, z, -y);   // K4A → Rhino swap
            p.Transform(floorTransform);
            return p;
        }

        private void StopTracker()
        {
            if (_tracker == null) return;
            try { _tracker.Dispose(); } catch { }
            _tracker = null;
        }

        // ─── Cleanup ────────────────────────────────────────────────────
        public override void RemovedFromDocument(GH_Document document)
        {
            base.RemovedFromDocument(document);
            StopTracker();
        }

        // ─── Boilerplate (unique GUID for this component) ───────────────
        protected override System.Drawing.Bitmap Icon => IconLoader.Load("body_icon.png");

        public override GH_Exposure Exposure => GH_Exposure.secondary;

        public override Guid ComponentGuid =>
            new Guid("c4e3a8f1-2b6d-4d5e-9a7c-8e1f3b5c7d9e");
    }
}