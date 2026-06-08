using System;
using System.Threading;
using Rhino;
using Grasshopper.Kernel;
using K4AdotNet.Sensor;
using Rhino.Geometry;

namespace AzureKinect.Components
{
    public class KinectDeviceComponent : GH_Component
    {
        // ?€?€?€ State that persists across SolveInstance calls ?€?€?€?€?€?€?€?€?€?€?€?€?€
        // Device lifetime is longer than a single solve, so these are
        // class member fields, not local variables.

        private Device _device;
        private Calibration _calibration;
        private DeviceConfiguration _configuration;
        private Transform _floorTransform = Transform.Identity;

        // For rising-edge detection of the Recalibrate button.
        private bool _previousRecalibrate;

        // ?€?€?€ Background capture thread ?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€
        private Thread _captureThread;
        private CancellationTokenSource _captureCancellation;
        private Capture _latestCapture;
        private readonly object _captureLock = new object();


        // ?€?€?€ Constructor ?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€
        public KinectDeviceComponent()
          : base("Kinect Device", "Kinect",
                 "Connects to and configures an Azure Kinect DK device.",
                 "Appendage", "Kinect")
        {
        }

        // ?€?€?€ Inputs ?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter(
                "Active", "On",
                "Activate / deactivate the device. Toggling off disposes the device.",
                GH_ParamAccess.item, false);

            pManager.AddIntegerParameter(
                "FPS", "FPS",
                "Frame rate: 0=5fps, 1=15fps, 2=30fps",
                GH_ParamAccess.item, 1);

            pManager.AddIntegerParameter(
                "Depth Mode", "Mode",
                "Depth mode: 0=Wide 1024, 1=Wide 512, 2=Narrow 640, 3=Narrow 320",
                GH_ParamAccess.item, 3);

            pManager.AddBooleanParameter(
                "Recalibrate", "Calib",
                "Trigger floor recalibration from IMU. Toggle false?’true to fire.",
                GH_ParamAccess.item, false);
        }

        // ?€?€?€ Outputs ?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter(
                "Kinect", "Dev",
                "Live Kinect device handle. Pass to Body Tracker / Point Cloud components.",
                GH_ParamAccess.item);

            pManager.AddTextParameter(
                "Status", "Status",
                "Current device status or error message.",
                GH_ParamAccess.item);
        }

        // ?€?€?€ Main solver ?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool active = false;
            int fpsIndex = 1;
            int depthModeIndex = 3;
            bool recalibrate = false;

            DA.GetData(0, ref active);
            DA.GetData(1, ref fpsIndex);
            DA.GetData(2, ref depthModeIndex);
            DA.GetData(3, ref recalibrate);

            string status;

            try
            {
                // ?€?€ Activate path
                if (active && _device == null)
                {
                    StartDevice(fpsIndex, depthModeIndex);
                    status = $"Device started (SN: {_device.SerialNumber})";
                }
                // ?€?€ Deactivate path
                else if (!active && _device != null)
                {
                    StopDevice();
                    status = "Device stopped";
                }
                // ?€?€ Recalibrate trigger (only on rising edge)
                else if (active && _device != null && recalibrate && !_previousRecalibrate)
                {
                    _floorTransform = ComputeFloorTransform();
                    status = "Floor recalibrated";
                }
                // ?€?€ Idle
                else
                {
                    status = _device != null ? "Device active" : "Device inactive";
                }
            }
            catch (Exception ex)
            {
                status = $"Error: {ex.Message}";
                StopDevice();   // Try to clean up on failure
            }

            _previousRecalibrate = recalibrate;

            // Emit outputs
            if (_device != null)
            {
                var handle = new GH_KinectDevice(_device)
                {
                    Calibration = _calibration,
                    Configuration = _configuration,
                    FloorTransform = _floorTransform,
                    AcquireCapture = AcquireLatestCapture,
                };
                DA.SetData(0, handle);
            }
            DA.SetData(1, status);
        }

        // ?€?€?€ Lifecycle helpers ?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€

        private void StartDevice(int fpsIndex, int depthModeIndex)
        {
            _device = Device.Open();

            _configuration = new DeviceConfiguration
            {
                CameraFps = IndexToFps(fpsIndex),
                DepthMode = IndexToDepthMode(depthModeIndex),
                ColorResolution = ColorResolution.Off,
                SynchronizedImagesOnly = false
            };

            _device.StartCameras(_configuration);
            _device.StartImu();

            _device.GetCalibration(
                _configuration.DepthMode,
                _configuration.ColorResolution,
                out _calibration);

            StartCaptureLoop();
            // Initial floor transform from current IMU readings
            _floorTransform = ComputeFloorTransform();
        }

        private void StopDevice()
        {
            if (_device == null) return;
            StopCaptureLoop();
            try { _device.StopImu(); } catch { }
            try { _device.StopCameras(); } catch { }
            try { _device.Dispose(); } catch { }

            _device = null;
            _calibration = default;
            _configuration = default;
            _floorTransform = Transform.Identity;
        }

        private void StartCaptureLoop()
        {
            if (_captureThread != null) return;
            _captureCancellation = new CancellationTokenSource();
            var token = _captureCancellation.Token;
            _captureThread = new Thread(() => CaptureLoop(token))
            {
                IsBackground = true,
                Name = "K4A Capture Loop"
            };
            _captureThread.Start();
        }

        private void StopCaptureLoop()
        {
            _captureCancellation?.Cancel();
            _captureThread?.Join();
            _captureThread = null;
            _captureCancellation?.Dispose();
            _captureCancellation = null;

            lock (_captureLock)
            {
                _latestCapture?.Dispose();
                _latestCapture = null;
            }
        }

        private void CaptureLoop(CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                Capture newCapture;
                try
                {
                    newCapture = _device.GetCapture();
                }
                catch
                {
                    // Device disposed or other error ??exit cleanly
                    break;
                }

                Capture old;
                lock (_captureLock)
                {
                    old = _latestCapture;
                    _latestCapture = newCapture;
                }
                old?.Dispose();
            }
        }

        public Capture AcquireLatestCapture()
        {
            lock (_captureLock)
            {
                return _latestCapture?.DuplicateReference();
            }
        }

        /// <summary>
        /// Reads IMU accelerometer samples, averages them to estimate gravity,
        /// then computes the rotation that aligns gravity with Rhino's -Z axis
        /// (i.e., makes the floor parallel to world XY plane).
        /// </summary>
        private Transform ComputeFloorTransform()
        {
            if (_device == null) return Transform.Identity;

            const int targetSamples = 100;
            float sumX = 0, sumY = 0, sumZ = 0;
            int collected = 0;

            var start = DateTime.UtcNow;
            var maxWait = TimeSpan.FromSeconds(2);

            while (collected < targetSamples && DateTime.UtcNow - start < maxWait)
            {
                if (_device.TryGetImuSample(out var sample, K4AdotNet.Timeout.FromMilliseconds(50)))
                {
                    sumX += sample.AccelerometerSample.X;
                    sumY += sample.AccelerometerSample.Y;
                    sumZ += sample.AccelerometerSample.Z;
                    collected++;
                }
            }

            if (collected < 10) return Transform.Identity;

            float avgX = sumX / collected;
            float avgY = sumY / collected;
            float avgZ = sumZ / collected;
            
            var rhinoGravity = new Vector3d(avgY, avgX, avgZ);
            if (!rhinoGravity.Unitize()) return Transform.Identity;

            // Rotation that maps current gravity to -Z (floor becomes horizontal)
            return Transform.Rotation(rhinoGravity, new Vector3d(0, 0, -1), Point3d.Origin);
        }

        private static FrameRate IndexToFps(int index)
        {
            switch (index)
            {
                case 0: return FrameRate.Five;
                case 2: return FrameRate.Thirty;
                default: return FrameRate.Fifteen;
            }
        }

        private static DepthMode IndexToDepthMode(int index)
        {
            switch (index)
            {
                case 0: return DepthMode.WideViewUnbinned;
                case 1: return DepthMode.WideView2x2Binned;
                case 2: return DepthMode.NarrowViewUnbinned;
                default: return DepthMode.NarrowView2x2Binned;
            }
        }

        // ?€?€?€ Cleanup when component removed from canvas ?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€
        public override void RemovedFromDocument(GH_Document document)
        {
            base.RemovedFromDocument(document);
            StopDevice();
        }

        // ?€?€?€ Boilerplate (KEEP THE EXISTING GUID) ?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€?€
        public override GH_Exposure Exposure => GH_Exposure.primary;
        protected override System.Drawing.Bitmap Icon => IconLoader.Load("AzureKinect24.png");

        public override Guid ComponentGuid =>
            new Guid("8996b8b7-b9cd-4c6a-a8d0-1c4b2c874de6");
    }
}





