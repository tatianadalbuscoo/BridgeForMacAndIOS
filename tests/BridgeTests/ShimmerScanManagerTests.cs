using ShimmerBridgeScan;
using Xunit;

namespace BridgeTests
{
    public class ShimmerScanManagerTests
    {
        [Fact]
        public void Entry_Defaults()
        {
            var e = new ShimmerScanManager.Entry();
            Assert.Equal("", e.Name);
            Assert.Equal("", e.Mac);
            Assert.Null(e.Rssi);
            Assert.False(e.IsPaired);
            Assert.Equal(ShimmerScanManager.DeviceType.Unknown, e.Type);
        }

        [Fact]
        public void Entry_Setters()
        {
            var e = new ShimmerScanManager.Entry
            {
                Name = "Shimmer3-ABC",
                Mac = "AA:BB:CC:DD:EE:FF",
                Rssi = -55,
                IsPaired = true,
                Type = ShimmerScanManager.DeviceType.IMU
            };

            Assert.Equal("Shimmer3-ABC", e.Name);
            Assert.Equal("AA:BB:CC:DD:EE:FF", e.Mac);
            Assert.Equal(-55, e.Rssi);
            Assert.True(e.IsPaired);
            Assert.Equal(ShimmerScanManager.DeviceType.IMU, e.Type);
        }
    }
}
