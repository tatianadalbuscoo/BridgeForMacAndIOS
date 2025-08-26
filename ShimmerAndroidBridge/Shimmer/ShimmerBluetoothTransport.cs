#if ANDROID
using System;
using ShimmerAPI;

namespace XR2Learn_ShimmerAPI.IMU.Android
{
    /// <summary>
    /// Adattatore: espone l’API ShimmerBluetooth usando IShimmerConnection (BT).
    /// </summary>
    internal sealed class ShimmerBluetoothTransport : ShimmerBluetooth
    {
        private readonly IShimmerConnection _conn;
        private string _address = "ANDROID-BT";

        public ShimmerBluetoothTransport(string deviceName, IShimmerConnection conn)
            : base(deviceName)
        {
            _conn = conn ?? throw new ArgumentNullException(nameof(conn));
        }

        // ===== Overrides richiesti dal base =====
        protected override bool IsConnectionOpen() => _conn.IsOpen;
        protected override void OpenConnection() => _conn.Open();
        protected override void CloseConnection() => _conn.Close();
        protected override int ReadByte() => _conn.ReadByte();
        protected override void WriteBytes(byte[] buffer, int index, int length) => _conn.WriteBytes(buffer, index, length);
        protected override void FlushConnection() => _conn.Flush();
        protected override void FlushInputConnection() => _conn.FlushInput();

        // >>> Implementazioni mancanti - indirizzo Shimmer <<<
        public override string GetShimmerAddress() => _address;
        public override void SetShimmerAddress(string address)
        {
            _address = string.IsNullOrWhiteSpace(address) ? "ANDROID-BT" : address.Trim();
        }

        // ===== Accessor pubblici utili =====
        public bool ConnectionOpen => IsConnectionOpen();                     // stato connessione
        public string FirmwareVersionFullNamePublic => FirmwareVersionFullName; // firmware

        // ===== Forward pubblici per Flush/FlushInput (servono al wrapper) =====
        public void Flush() => FlushConnection();
        public void FlushInput() => FlushInputConnection();
    }
}
#endif
