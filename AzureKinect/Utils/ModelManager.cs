using System;
using System.IO;
using System.Net;

namespace AzureKinect
{
    /// <summary>
    /// Finds or downloads the Body Tracking ONNX model.
    ///
    /// Resolution order (first hit wins):
    ///   1. Plugin install folder  -- bundled in dev builds, or dropped manually by user
    ///   2. Local cache folder      -- %LOCALAPPDATA%\AzureKinect\
    ///   3. Download from GitHub Release into the cache (one-time, ~159 MB)
    ///
    /// The model is shipped separately from the .yak because yak.rhino3d.com
    /// can't reliably accept packages above ~30 MB (Heroku 30-second upload limit).
    /// </summary>
    internal static class ModelManager
    {
        private const string ModelFileName = "dnn_model_2_0_op11.onnx";

        // GitHub Release tag must match the version segment of this URL.
        private const string DownloadUrl =
            "https://github.com/AndrewJinhoAhn/AzureKinectGH/releases/download/v1.1.0/" + ModelFileName;

        private static readonly string CacheDir = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            "AzureKinect");

        public static readonly string CachePath = Path.Combine(CacheDir, ModelFileName);

        /// <summary>
        /// Returns true if the model is already on disk (no download needed).
        /// Lets callers warn the user before a long blocking call.
        /// </summary>
        public static bool IsAvailableLocally()
        {
            var pluginDir = AzureKinectInfo.PluginDirectory;
            if (!string.IsNullOrEmpty(pluginDir) &&
                File.Exists(Path.Combine(pluginDir, ModelFileName)))
                return true;

            return File.Exists(CachePath);
        }

        /// <summary>
        /// Returns the path to a usable ONNX model file, downloading if needed.
        /// Blocks the calling thread until the download finishes (~30s on broadband).
        /// Throws on failure with a message explaining the manual fallback.
        /// </summary>
        public static string GetModelPath()
        {
            // 1. Plugin folder
            var pluginDir = AzureKinectInfo.PluginDirectory;
            if (!string.IsNullOrEmpty(pluginDir))
            {
                var bundled = Path.Combine(pluginDir, ModelFileName);
                if (File.Exists(bundled)) return bundled;
            }

            // 2. Cached download
            if (File.Exists(CachePath)) return CachePath;

            // 3. Download
            Directory.CreateDirectory(CacheDir);
            var tmpPath = CachePath + ".part";

            try
            {
                // GitHub requires TLS 1.2+; default may be TLS 1.0 in older Rhino.
                ServicePointManager.SecurityProtocol |=
                    SecurityProtocolType.Tls12 | SecurityProtocolType.Tls13;

                // Best-effort user feedback (Rhino is loaded by the time we get here).
                try
                {
                    Rhino.RhinoApp.WriteLine(
                        "AzureKinect: first-time body tracking setup -- " +
                        "downloading model (~159 MB) from GitHub. This only happens once.");
                }
                catch { /* ignore — log fallback below */ }

                using (var client = new WebClient())
                {
                    client.DownloadFile(DownloadUrl, tmpPath);
                }
                File.Move(tmpPath, CachePath);

                try { Rhino.RhinoApp.WriteLine("AzureKinect: model downloaded successfully."); }
                catch { }

                return CachePath;
            }
            catch (Exception ex)
            {
                try { if (File.Exists(tmpPath)) File.Delete(tmpPath); } catch { }
                throw new InvalidOperationException(
                    "Body tracking model download failed.\n\n" +
                    "Manually download:\n  " + DownloadUrl + "\n" +
                    "and place it at:\n  " + CachePath + "\n\n" +
                    "Inner error: " + ex.Message, ex);
            }
        }
    }
}
