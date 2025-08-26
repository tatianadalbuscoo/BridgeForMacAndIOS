#if ANDROID
using System.IO;

namespace XR2Learn_ShimmerAPI.IMU.Android
{
    /// <summary>Astrazione di trasporto per ShimmerBluetooth (solo Android).</summary>
    internal interface IShimmerConnection
    {
        bool IsOpen { get; }
        void Open();
        void Close();
        int ReadByte();
        void WriteBytes(byte[] buffer, int index, int length);
        void Flush();
        void FlushInput();
    }
}
#endif
