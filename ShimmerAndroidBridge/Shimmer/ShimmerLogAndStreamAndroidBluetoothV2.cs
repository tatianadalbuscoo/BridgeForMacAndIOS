#if ANDROID
using System;
using System.Threading;
using System.Threading.Tasks;
using ShimmerAPI;

namespace XR2Learn_ShimmerAPI.IMU.Android
{
    /// <summary>
    /// Wrapper Android con stesse API del V2 Windows (cambia solo il trasporto).
    /// </summary>
    public sealed class ShimmerLogAndStreamAndroidBluetoothV2
    {
        private readonly ShimmerBluetoothTransport _core;
        private readonly string _mac;

        public event EventHandler? UICallback
        {
            add { _core.UICallback += value; }
            remove { _core.UICallback -= value; }
        }

        public ShimmerLogAndStreamAndroidBluetoothV2(string deviceName, string mac)
        {
            _mac = (mac ?? string.Empty).Trim();
            var transport = new AndroidBluetoothConnection(_mac);
            _core = new ShimmerBluetoothTransport(deviceName, transport);
            _core.SetShimmerAddress(_mac);
        }

        public bool IsConnected() => _core.ConnectionOpen;


        public void Connect()
        {
            _core.Connect();
        }

        public void Disconnect()
        {
            try { StopStreaming(); } catch { }
            try { _core.Disconnect(); } catch { }
        }




        public void StartStreaming() => _core.StartStreaming();
        public void StopStreaming()  => _core.StopStreaming();

        // ===== NUOVO: forward per flush dei buffer =====
        public void Flush()      => _core.Flush();
        public void FlushInput() => _core.FlushInput();

        // Deleghe 1:1 al core (come Windows)
        public void WriteSensors(int bitmap) => _core.WriteSensors(bitmap);
        public void WriteAccelRange(int r) => _core.WriteAccelRange(r);
        public void WriteGyroRange(int r) => _core.WriteGyroRange(r);
        public void WriteGSRRange(int r) => _core.WriteGSRRange(r);
        public void SetLowPowerAccel(bool on) => _core.SetLowPowerAccel(on);
        public void SetLowPowerGyro(bool on) => _core.SetLowPowerGyro(on);
        public void WriteInternalExpPower(int on) => _core.WriteInternalExpPower(on);
        public void WriteSamplingRate(int hz) => _core.WriteSamplingRate(hz);
        public void Inquiry() => _core.Inquiry();
        public void ReadCalibrationParameters(string which) => _core.ReadCalibrationParameters(which);

        public string GetFirmwareVersionFullName() => _core.FirmwareVersionFullNamePublic;
        public string GetShimmerAddress() => _core.GetShimmerAddress();
    }
}
#endif
