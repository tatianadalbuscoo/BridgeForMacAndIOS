/*
 * Android-specific wrapper that mimics the Windows V2 API.
 * Internally uses ShimmerBluetoothTransport with an AndroidBluetoothConnection
 * to manage Bluetooth RFCOMM communication.
 *
 * Purpose: provide API parity with Windows V2 so higher-level code can run
 * unchanged across platforms. Delegates all sensor configuration and streaming
 * commands to the core transport.
 */


#if ANDROID
using System;
using System.Threading;
using System.Threading.Tasks;
using ShimmerAPI;

namespace ShimmerSDK.Android
{

    /// <summary>
    /// Android wrapper that mirrors the older Windows V2 API while using the Android
    /// Bluetooth transport under the hood.
    /// </summary>
    public sealed class ShimmerLogAndStreamAndroidBluetoothV2
    {
        private readonly ShimmerBluetoothTransport _core;
        private readonly string _mac;


        /// <summary>
        /// UI/event callback forwarded to the underlying <see cref="ShimmerBluetoothTransport"/>.
        /// </summary>
        public event EventHandler? UICallback
        {
            add { _core.UICallback += value; }
            remove { _core.UICallback -= value; }
        }


        /// <summary>
        /// Creates the V2-style Android wrapper around a Bluetooth transport.
        /// </summary>
        /// <param name="deviceName">Logical device name exposed to the SDK.</param>
        /// <param name="mac">Bluetooth MAC address of the target Shimmer device.</param>
        public ShimmerLogAndStreamAndroidBluetoothV2(string deviceName, string mac)
        {
            _mac = (mac ?? string.Empty).Trim();
            var transport = new AndroidBluetoothConnection(_mac);
            _core = new ShimmerBluetoothTransport(deviceName, transport);
            _core.SetShimmerAddress(_mac);
        }


        /// <summary>
        /// Returns whether the underlying connection is open.
        /// </summary>
        /// <returns><c>true</c> if the connection is open; otherwise <c>false</c>.</returns>
        public bool IsConnected() => _core.ConnectionOpen;


        /// <summary>
        /// Opens the connection to the Shimmer device.
        /// </summary>
        public void Connect()
        {
            _core.Connect();
        }


        /// <summary>
        /// Stops streaming (if active) and closes the connection.
        /// </summary>
        public void Disconnect()
        {
            try { StopStreaming(); } catch { }
            try { _core.Disconnect(); } catch { }
        }


        /// <summary>
        /// Starts data streaming from the device.
        /// </summary>
        public void StartStreaming() => _core.StartStreaming();


        /// <summary>
        /// Stops data streaming from the device.
        /// </summary>
        public void StopStreaming()  => _core.StopStreaming();


        // ----- Buffer maintenance (forward to core; no-op on Android RFCOMM) -----


        /// <summary>
        /// Flushes pending output if supported by the transport (no-op on Android).
        /// </summary>
        public void Flush()      => _core.Flush();


        /// <summary>
        /// Clears pending input if supported by the transport (no-op on Android).
        /// </summary>
        public void FlushInput() => _core.FlushInput();


        // ----- 1:1 delegates to core (Windows V2 parity) -----
        //
        // NOTE: These methods are not implemented in ShimmerBluetoothTransport directly.
        // They come from the base class ShimmerBluetooth (part of ShimmerAPI),
        // which provides all firmware commands (sensor config, ranges, inquiry, etc.).
        // This wrapper simply re-exposes them to maintain API parity with the Windows V2 API.


        /// <summary>
        /// Enables or disables sensors on the Shimmer device using a bitmask.
        /// </summary>
        /// <param name="bitmap">Bitmask of sensors to enable/disable.</param>
        public void WriteSensors(int bitmap) => _core.WriteSensors(bitmap);


        /// <summary>
        /// Sets the full-scale range of the accelerometer.
        /// </summary>
        /// <param name="r">Accelerometer range value.</param>
        public void WriteAccelRange(int r) => _core.WriteAccelRange(r);

        
        /// <summary>
        /// Sets the full-scale range of the gyroscope.
        /// </summary>
        /// <param name="r">Gyroscope range value.</param>
        public void WriteGyroRange(int r) => _core.WriteGyroRange(r);


        /// <summary>
        /// Enables or disables low-power mode for the accelerometer.
        /// </summary>
        /// <param name="on"><c>true</c> to enable low-power mode, <c>false</c> to disable.</param>
        public void SetLowPowerAccel(bool on) => _core.SetLowPowerAccel(on);


        /// <summary>
        /// Enables or disables low-power mode for the gyroscope.
        /// </summary>
        /// <param name="on"><c>true</c> to enable low-power mode, <c>false</c> to disable.</param>
        public void SetLowPowerGyro(bool on) => _core.SetLowPowerGyro(on);


        /// <summary>
        /// Turns the internal expansion power rail on or off.
        /// </summary>
        /// <param name="on"> 0 = off, 1 = on (firmware-dependent).</param>
        public void WriteInternalExpPower(int on) => _core.WriteInternalExpPower(on);


        /// <summary>
        /// Requests calibration parameters from the device.
        /// </summary>
        /// <param name="which">
        /// Scope of the request, e.g. "All" to fetch every available calibration,
        /// or a specific sensor name depending on firmware support.
        /// </param>
        public void ReadCalibrationParameters(string which) => _core.ReadCalibrationParameters(which);


        /// <summary>
        /// Sets the sampling rate of the Shimmer device.
        /// </summary>
        /// <param name="hz">Desired sampling rate in Hz.</param>
        public void WriteSamplingRate(int hz) => _core.WriteSamplingRate(hz);


        /// <summary>
        /// Sends an Inquiry command to the device to refresh packet size,
        /// signal name mappings, and other runtime metadata.
        /// </summary>
        public void Inquiry() => _core.Inquiry();


        // These two are implemented in ShimmerBluetoothTransport (not in the base ShimmerBluetooth).


        /// <summary>
        /// Gets the full firmware version string of the connected Shimmer device.
        /// </summary>
        /// <returns>A human-readable firmware version string.</returns>
        public string GetFirmwareVersionFullName() => _core.FirmwareVersionFullNamePublic;


        /// <summary>
        /// Gets the address (Bluetooth MAC) of the connected Shimmer device.
        /// </summary>
        /// <returns>The Bluetooth MAC address string.</returns>
        public string GetShimmerAddress() => _core.GetShimmerAddress();
    }
}
#endif
