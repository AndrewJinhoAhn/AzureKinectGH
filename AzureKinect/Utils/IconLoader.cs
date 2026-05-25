using System.Collections.Generic;
using System.Drawing;
using System.Reflection;

namespace AzureKinect
{
    /// <summary>
    /// Loads embedded PNG icons from the assembly's Resources folder.
    /// Caches by name so each icon is only constructed once.
    /// </summary>
    internal static class IconLoader
    {
        private static readonly Dictionary<string, Bitmap> _cache = new Dictionary<string, Bitmap>();
        private static readonly Assembly _assembly = typeof(IconLoader).Assembly;

        public static Bitmap Load(string fileName)
        {
            if (_cache.TryGetValue(fileName, out var cached)) return cached;

            // Embedded resource name format: {DefaultNamespace}.{FolderPath}.{FileName}
            var resourceName = $"AzureKinect.Resources.{fileName}";

            using (var stream = _assembly.GetManifestResourceStream(resourceName))
            {
                if (stream == null) return null;
                var bmp = new Bitmap(stream);
                _cache[fileName] = bmp;
                return bmp;
            }
        }
    }
}