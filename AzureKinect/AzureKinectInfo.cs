using Grasshopper;
using Grasshopper.Kernel;
using System;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;

namespace AzureKinect
{
    public class AzureKinectInfo : GH_AssemblyInfo
    {
        // ─── Native interop ────────────────────────────────────────────
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool SetDllDirectory(string lpPathName);

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern IntPtr LoadLibrary(string lpFileName);

        // Plugin folder — used by other components (e.g., BodyTracker).
        internal static string PluginDirectory { get; private set; }

        // ─── Static constructor ────────────────────────────────────────
        // Runs ONCE when GH first loads this assembly. Sets up native DLL
        // search paths and tells K4AdotNet where to find body tracking runtime.
        static AzureKinectInfo()
        {
            try
            {
                PluginDirectory = Path.GetDirectoryName(typeof(AzureKinectInfo).Assembly.Location);
                Log($"PluginDirectory: {PluginDirectory ?? "(null)"}");

                if (string.IsNullOrEmpty(PluginDirectory))
                {
                    Log("PluginDirectory empty, abort init");
                    return;
                }

                bool setDllResult = SetDllDirectory(PluginDirectory);
                Log($"SetDllDirectory result: {setDllResult}");

                var current = Environment.GetEnvironmentVariable("PATH") ?? "";
                Environment.SetEnvironmentVariable("PATH", PluginDirectory + ";" + current);

                string[] preload = {
                    "depthengine_2_0.dll",
                    "k4a.dll",
                    "directml.dll",
                    "onnxruntime.dll",
                    "onnxruntime_providers_shared.dll",
                    "k4abt.dll",
                };

                foreach (var dll in preload)
                {
                    var path = Path.Combine(PluginDirectory, dll);
                    if (!File.Exists(path))
                    {
                        Log($"FILE MISSING: {dll}");
                        continue;
                    }
                    var h = LoadLibrary(path);
                    if (h == IntPtr.Zero)
                    {
                        int err = Marshal.GetLastWin32Error();
                        Log($"LoadLibrary FAILED for {dll}: Win32 error {err}");
                    }
                    else
                    {
                        Log($"Loaded OK: {dll}");
                    }
                }

                // Tell K4AdotNet where body tracking runtime lives.
                // Use reflection so yak inspection (which can't load K4AdotNet)
                // doesn't fail at JIT-compile time.
                try
                {
                    var sdkType = Type.GetType("K4AdotNet.Sdk, K4AdotNet");
                    if (sdkType != null)
                    {
                        var prop = sdkType.GetProperty(
                            "BodyTrackingRuntimePath",
                            System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static);
                        prop?.SetValue(null, PluginDirectory);
                        Log($"BodyTrackingRuntimePath set via reflection to: {PluginDirectory}");
                    }
                    else
                    {
                        Log("K4AdotNet.Sdk type not found, skipping path config");
                    }
                }
                catch (Exception ex)
                {
                    Log($"Failed to set BodyTrackingRuntimePath: {ex.Message}");
                }
            }
            catch (Exception ex)
            {
                Log($"Static ctor exception: {ex}");
            }
        }

        // ─── File-based logging (independent of Rhino) ─────────────────
        private static void Log(string msg)
        {
            try
            {
                var logPath = Path.Combine(
                    Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
                    "AzureKinect_init.log");
                File.AppendAllText(logPath, $"[{DateTime.Now:HH:mm:ss}] {msg}\r\n");
            }
            catch { }
        }

        // ─── Standard GH_AssemblyInfo overrides ────────────────────────
        public override string Name => "AzureKinect";
        public override Bitmap Icon => IconLoader.Load("plugin_icon.png");
        public override string Description => "Azure Kinect DK integration for Grasshopper";
        public override Guid Id => new Guid("9bcdcead-87c5-43a4-a653-ee9e253b94d2");
        public override string AuthorName => "Andrew Jinho Ahn";
        public override string AuthorContact => "";
        public override string AssemblyVersion => GetType().Assembly.GetName().Version.ToString();
    }
}