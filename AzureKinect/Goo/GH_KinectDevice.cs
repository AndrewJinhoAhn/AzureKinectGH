using System;
using Grasshopper.Kernel.Types;
using K4AdotNet.Sensor;
using RhinoTransform = Rhino.Geometry.Transform;

namespace AzureKinect
{
    /// <summary>
    /// Grasshopper goo wrapper around a live Azure Kinect device handle.
    /// Carries the device, its calibration, and a gravity-derived floor transform
    /// so downstream components (Body Tracker, Point Cloud) can share a single
    /// connection by just receiving this one object.
    ///
    /// Reference semantics: duplicating this goo does NOT open a new device.
    /// All wrappers point at the same underlying Device instance.
    /// </summary>
    public class GH_KinectDevice : GH_Goo<Device>
    {
        // ─── Extra state carried alongside the Device ──────────────────────

        /// <summary>
        /// Camera calibration. Required for depth-to-pointcloud projection
        /// and for initializing the Body Tracker.
        /// </summary>
        public Calibration Calibration { get; set; }

        /// <summary>
        /// Transform that rotates raw Kinect coordinates so that gravity
        /// points along -Z (i.e., the floor becomes parallel to world XY).
        /// Default is identity (no correction applied yet).
        /// </summary>
        public RhinoTransform FloorTransform { get; set; } = RhinoTransform.Identity;

        /// <summary>
        /// The DeviceConfiguration used to start the cameras
        /// (FPS, depth mode, etc.). Carried so downstream components
        /// don't need to re-query the device.
        /// </summary>
        public DeviceConfiguration Configuration { get; set; }

        /// <summary>
        /// Returns a referenced copy of the latest capture, or null if not running.
        /// Caller must dispose the returned Capture when done.
        /// Set by KinectDeviceComponent when it creates this handle.
        /// </summary>
        public Func<Capture> AcquireCapture { get; set; }

        // ─── Constructors ──────────────────────────────────────────────────

        public GH_KinectDevice() : base() { }
        public GH_KinectDevice(Device device) : base(device) { }

        // ─── IGH_Goo contract ──────────────────────────────────────────────

        public override bool IsValid
        {
            get
            {
                if (Value == null) return false;
                try
                {
                    // Touching SerialNumber forces a liveness check;
                    // disposed devices throw here.
                    var _ = Value.SerialNumber;
                    return true;
                }
                catch
                {
                    return false;
                }
            }
        }

        public override string TypeName => "Kinect Device";
        public override string TypeDescription =>
            "Azure Kinect device handle (device + calibration + floor transform).";

        public override IGH_Goo Duplicate()
        {
            // Share the same underlying Device — reference semantics.
            // Calibration and Configuration are also shared (no deep copy);
            // FloorTransform is a struct so it gets value-copied automatically.
            return new GH_KinectDevice
            {
                Value = this.Value,
                Calibration = this.Calibration,
                FloorTransform = this.FloorTransform,
                Configuration = this.Configuration,
                AcquireCapture = this.AcquireCapture,
            };
        }

        public override string ToString()
        {
            if (Value == null) return "Kinect Device (null)";
            if (!IsValid) return "Kinect Device (disposed)";
            return $"Kinect Device (SN: {Value.SerialNumber})";
        }
    }
}