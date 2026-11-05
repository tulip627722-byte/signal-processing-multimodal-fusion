using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

namespace Acquire
{
    internal static class OrbbecFactoryCalibration
    {
        public static int Dump(string outputPath)
        {
            OpenNi2Locator locator = OpenNi2Locator.Locate();
            if (!locator.NativeFound)
            {
                ConsoleIo.WriteLine(locator.DescribeMissing());
                return 2;
            }

            locator.EnsureDriverRepository(delegate(string line) { ConsoleIo.WriteLine(line); });
            using (OpenNi2NativeApi api = OpenNi2NativeApi.Open(locator.NativeDll))
            {
                List<OniDeviceInfo> devices = api.ListDevices();
                if (devices.Count == 0)
                {
                    ConsoleIo.WriteLine("没有检测到 OpenNI2 相机");
                    return 1;
                }

                OniDeviceInfo listed = devices[0];
                IntPtr device = IntPtr.Zero;
                IntPtr depthStream = IntPtr.Zero;
                IntPtr colorStream = IntPtr.Zero;
                try
                {
                    device = api.OpenDevice(listed.Uri);
                    OniDeviceInfo info = api.GetDeviceInfo(device, listed);
                    // 官方 ROS 驱动在 setupVideoMode() 之后才读取 EEPROM；部分固件在未建流时返回全 NaN。
                    depthStream = PrepareStream(api, device, Oni2.SensorDepth, Oni2.PixelDepth1Mm);
                    colorStream = PrepareStream(api, device, Oni2.SensorColor, Oni2.PixelRgb888);
                    ObCameraParams p = api.GetFactoryCameraParams(device);
                    string validationError = Validate(p);
                    if (validationError != null)
                        throw new InvalidOperationException("相机返回了无效的出厂标定：" + validationError +
                                                            "。原始值：" + RawSummary(p));

                    JsonObject root = BuildJson(info, p);
                    string json = MiniJson.Serialize(root);
                    if (!string.IsNullOrEmpty(outputPath))
                    {
                        string fullPath = Path.GetFullPath(outputPath);
                        string directory = Path.GetDirectoryName(fullPath);
                        if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
                        File.WriteAllText(fullPath, json, new UTF8Encoding(false));
                        ConsoleIo.WriteLine("出厂标定已保存：" + fullPath);
                    }

                    PrintSummary(info, p);
                    return 0;
                }
                finally
                {
                    if (colorStream != IntPtr.Zero && api.StreamDestroy != null) api.StreamDestroy(colorStream);
                    if (depthStream != IntPtr.Zero && api.StreamDestroy != null) api.StreamDestroy(depthStream);
                    if (device != IntPtr.Zero && api.DeviceClose != null) api.DeviceClose(device);
                }
            }
        }

        private static IntPtr PrepareStream(OpenNi2NativeApi api, IntPtr device, int sensorType, int preferredFormat)
        {
            List<OniVideoMode> modes = api.GetSupportedModes(device, sensorType);
            if (modes.Count == 0) return IntPtr.Zero;
            OniVideoMode selected = modes[0];
            foreach (OniVideoMode mode in modes)
            {
                if (mode.ResolutionX == 640 && mode.ResolutionY == 480 && mode.PixelFormat == preferredFormat)
                {
                    selected = mode;
                    break;
                }
            }
            IntPtr stream = api.CreateStream(device, sensorType);
            api.TrySetVideoMode(stream, selected.PixelFormat, selected.ResolutionX, selected.ResolutionY, selected.Fps);
            return stream;
        }

        private static JsonObject BuildJson(OniDeviceInfo info, ObCameraParams p)
        {
            JsonObject device = new JsonObject();
            device.Add("name", info.Name);
            device.Add("vendor", info.Vendor);
            device.Add("serialNumber", info.SerialNumber);
            device.Add("uri", info.Uri);
            device.Add("usbVendorId", (int)info.UsbVendorId);
            device.Add("usbProductId", (int)info.UsbProductId);

            JsonObject color = new JsonObject();
            color.Add("intrinsicsFxFyCxCy", p.RightIntrinsics);
            color.Add("cameraMatrix", CameraMatrix(p.RightIntrinsics));
            color.Add("distortionRawK1K2K3P1P2", p.RightDistortion);
            color.Add("distortionOpenCvK1K2P1P2K3", OpenCvDistortion(p.RightDistortion));

            JsonObject depth = new JsonObject();
            depth.Add("intrinsicsFxFyCxCy", p.LeftIntrinsics);
            depth.Add("cameraMatrix", CameraMatrix(p.LeftIntrinsics));
            depth.Add("distortionRawK1K2K3P1P2", p.LeftDistortion);
            depth.Add("distortionOpenCvK1K2P1P2K3", OpenCvDistortion(p.LeftDistortion));

            JsonObject extrinsics = new JsonObject();
            extrinsics.Add("rotationRowMajor", p.RightToLeftRotation);
            extrinsics.Add("translation", p.RightToLeftTranslation);
            extrinsics.Add("direction", "color/right to depth/left (r2l), units as stored by device");

            JsonObject root = new JsonObject();
            root.Add("schema", "orbbec-openni2-factory-calibration-v1");
            root.Add("readAtLocal", DateTime.Now.ToString("yyyy-MM-ddTHH:mm:ss.fffzzz", CultureInfo.InvariantCulture));
            root.Add("source", "Device EEPROM via OpenNI2 OBEXTENSION_ID_CAM_PARAMS (property 14)");
            root.Add("nativeStructBytes", 120);
            root.Add("device", device);
            root.Add("colorRight", color);
            root.Add("depthLeft", depth);
            root.Add("colorToDepth", extrinsics);
            return root;
        }

        private static float[] CameraMatrix(float[] k)
        {
            return new float[] { k[0], 0, k[2], 0, k[1], k[3], 0, 0, 1 };
        }

        private static float[] OpenCvDistortion(float[] raw)
        {
            // Orbbec raw r_k/l_k is [k1,k2,k3,p1,p2]. ROS/OpenCV expects [k1,k2,p1,p2,k3].
            return new float[] { raw[0], raw[1], raw[3], raw[4], raw[2] };
        }

        private static string Validate(ObCameraParams p)
        {
            if (p.LeftIntrinsics == null || p.RightIntrinsics == null || p.RightToLeftRotation == null ||
                p.RightToLeftTranslation == null || p.LeftDistortion == null || p.RightDistortion == null)
                return "字段缺失";
            if (!PlausibleIntrinsics(p.LeftIntrinsics)) return "深度内参不合理";
            if (!PlausibleIntrinsics(p.RightIntrinsics)) return "彩色内参不合理";
            if (!AllFinite(p.RightToLeftRotation) || !AllFinite(p.RightToLeftTranslation) ||
                !AllFinite(p.LeftDistortion) || !AllFinite(p.RightDistortion)) return "包含 NaN/Infinity";
            return null;
        }

        private static bool PlausibleIntrinsics(float[] k)
        {
            return k.Length == 4 && AllFinite(k) && k[0] > 10 && k[0] < 10000 &&
                   k[1] > 10 && k[1] < 10000 && k[2] > -1000 && k[2] < 5000 &&
                   k[3] > -1000 && k[3] < 5000;
        }

        private static bool AllFinite(float[] values)
        {
            foreach (float value in values)
                if (float.IsNaN(value) || float.IsInfinity(value)) return false;
            return true;
        }

        private static void PrintSummary(OniDeviceInfo info, ObCameraParams p)
        {
            ConsoleIo.WriteLine("设备：" + info.Name + (string.IsNullOrEmpty(info.SerialNumber) ? "" : "  SN:" + info.SerialNumber));
            ConsoleIo.WriteLine("RGB 内参 [fx fy cx cy]：" + Join(p.RightIntrinsics));
            ConsoleIo.WriteLine("RGB 畸变 [k1 k2 p1 p2 k3]：" + Join(OpenCvDistortion(p.RightDistortion)));
            ConsoleIo.WriteLine("深度内参 [fx fy cx cy]：" + Join(p.LeftIntrinsics));
            ConsoleIo.WriteLine("RGB->深度 平移：" + Join(p.RightToLeftTranslation));
        }

        private static string RawSummary(ObCameraParams p)
        {
            return "leftK=[" + Join(p.LeftIntrinsics) + "] rightK=[" + Join(p.RightIntrinsics) +
                   "] r2lR=[" + Join(p.RightToLeftRotation) + "] r2lT=[" +
                   Join(p.RightToLeftTranslation) + "] leftD=[" + Join(p.LeftDistortion) +
                   "] rightD=[" + Join(p.RightDistortion) + "]";
        }

        private static string Join(float[] values)
        {
            string[] text = new string[values.Length];
            for (int i = 0; i < values.Length; i++)
                text[i] = values[i].ToString("0.#########", CultureInfo.InvariantCulture);
            return string.Join(" ", text);
        }
    }
}
