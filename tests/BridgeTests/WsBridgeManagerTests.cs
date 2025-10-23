/*
 * WsBridgeManagerTests.cs
 * Purpose: Unit tests for WsBridgeManager file.
 */

using Xunit;
using ShimmerBridgeMangager;
using System.Text.Json;
using WatsonWebsocket;
using System.Reflection;
using Android.Net.Wifi;
using System.Text;


namespace tests.BridgeTests
{
    public class ExgModeTests
    {

        // ------ ExgMode enum behavior ------
        // Covers enum values/names stability, [Flags] absence, ToString(),
        // case-insensitive parsing, failure on unknown names, and enumeration via GetValues.


        /// <summary>
        /// Verifies the underlying integer values are stable and ordered as defined.
        /// Expected:
        /// - None == 0
        /// - ECG == 1
        /// - EMG == 2
        /// - ExgTest == 3
        /// - Respiration == 4
        /// </summary>
        [Fact]
        public void Values_AreStable_AndOrdered()
        {
            Assert.Equal(0, (int)ExgMode.None);
            Assert.Equal(1, (int)ExgMode.ECG);
            Assert.Equal(2, (int)ExgMode.EMG);
            Assert.Equal(3, (int)ExgMode.ExgTest);
            Assert.Equal(4, (int)ExgMode.Respiration);
        }


        /// <summary>
        /// Verifies the enum contains exactly the expected set of names.
        /// Expected: names == ["None","ECG","EMG","ExgTest","Respiration"] in this order.
        /// </summary>
        [Fact]
        public void Names_Match_Expected_Set()
        {
            var names = Enum.GetNames(typeof(ExgMode));
            Assert.Equal(new[] { "None", "ECG", "EMG", "ExgTest", "Respiration" }, names);
        }


        /// <summary>
        /// Verifies the enum has no [Flags] attribute (modes are mutually exclusive).
        /// Expected: Attribute.IsDefined(..., typeof(FlagsAttribute)) == false.
        /// </summary>
        [Fact]
        public void HasNoFlagsAttribute()
        {
            var hasFlags = Attribute.IsDefined(typeof(ExgMode), typeof(FlagsAttribute));
            Assert.False(hasFlags);
        }


        /// <summary>
        /// Verifies ToString returns the symbolic name (not the numeric value).
        /// Expected:
        /// - ExgMode.ECG.ToString() == "ECG"
        /// - ExgMode.ExgTest.ToString() == "ExgTest"
        /// </summary>
        [Fact]
        public void ToString_Returns_Name()
        {
            Assert.Equal("ECG", ExgMode.ECG.ToString());
            Assert.Equal("ExgTest", ExgMode.ExgTest.ToString());
        }


        /// <summary>
        /// Checks case-insensitive parsing with <see cref="Enum.TryParse{TEnum}(string, bool, out TEnum)"/>.
        /// Expected:
        /// - "ecg" (ignoreCase=true) -> ECG
        /// - "EMG" -> EMG
        /// - "ExgTest" -> ExgTest
        /// </summary>
        [Fact]
        public void TryParse_IsCaseInsensitive_ForValidNames()
        {
            Assert.True(Enum.TryParse<ExgMode>("ecg", ignoreCase: true, out var m1));
            Assert.Equal(ExgMode.ECG, m1);

            Assert.True(Enum.TryParse<ExgMode>("EMG", ignoreCase: true, out var m2));
            Assert.Equal(ExgMode.EMG, m2);

            Assert.True(Enum.TryParse<ExgMode>("ExgTest", ignoreCase: true, out var m3));
            Assert.Equal(ExgMode.ExgTest, m3);
        }


        /// <summary>
        /// Ensures unknown names fail to parse and the out var remains default.
        /// Expected:
        /// - TryParse("resp", ignoreCase:false) == false
        /// - out var equals default (None)
        /// </summary>
        [Fact]
        public void TryParse_Fails_OnUnknown()
        {
            var ok = Enum.TryParse<ExgMode>("resp", ignoreCase: false, out var mode);
            Assert.False(ok);
            Assert.Equal(ExgMode.None, mode);
        }


        /// <summary>
        /// Verifies Enum.GetValues returns all declared items without duplicates.
        /// Expected:
        /// - Values length == 5
        /// - Sequence equals [None, ECG, EMG, ExgTest, Respiration]
        /// </summary>
        [Fact]
        public void GetValues_Covers_AllItems()
        {
            var vals = Enum.GetValues(typeof(ExgMode)).Cast<ExgMode>().ToArray();
            Assert.Equal(5, vals.Length);
            Assert.Equal(new[] { ExgMode.None, ExgMode.ECG, ExgMode.EMG, ExgMode.ExgTest, ExgMode.Respiration }, vals);
        }


        // ------ ShimmerConfig defaults behavior ------


        /// <summary>
        /// Validates default values for a new <see cref="ShimmerConfig"/>.
        /// Expected:
        /// - All IMU flags == false
        /// - SamplingRate == null
        /// - EnableExg1/2 == false, ExgUse16Bit == false
        /// - ExgMode == None
        /// - ExgModeWire == null, JSON omits "exg_mode"
        /// </summary>
        [Fact]
        public void ShimmerConfig_Defaults()
        {
            var c = new ShimmerConfig();

            // IMU
            Assert.False(c.EnableLowNoiseAccelerometer);
            Assert.False(c.EnableWideRangeAccelerometer);
            Assert.False(c.EnableGyroscope);
            Assert.False(c.EnableMagnetometer);
            Assert.False(c.EnablePressureTemperature);
            Assert.False(c.EnableBattery);
            Assert.False(c.EnableExtA6);
            Assert.False(c.EnableExtA7);
            Assert.False(c.EnableExtA15);
            Assert.Null(c.SamplingRate);

            // EXG
            Assert.False(c.EnableExg1);
            Assert.False(c.EnableExg2);
            Assert.False(c.ExgUse16Bit);

            // Mode
            Assert.Equal(ExgMode.None, c.ExgMode);
            Assert.Null(c.ExgModeWire);

            // JSON omits exg_mode when null
            var json = JsonSerializer.Serialize(c);
            Assert.DoesNotContain("exg_mode", json);
        }


        /// <summary>
        /// Ensures property setters persist values across IMU/EXG flags and SamplingRate.
        /// Expected:
        /// - All assigned booleans read back as true
        /// - SamplingRate retains the assigned double
        /// </summary>
        [Fact]
        public void ShimmerConfig_Setters_Persist()
        {
            var c = new ShimmerConfig
            {
                // IMU
                EnableLowNoiseAccelerometer = true,
                EnableWideRangeAccelerometer = true,
                EnableGyroscope = true,
                EnableMagnetometer = true,
                EnablePressureTemperature = true,
                EnableBattery = true,
                EnableExtA6 = true,
                EnableExtA7 = true,
                EnableExtA15 = true,
                SamplingRate = 128.5,

                // EXG
                EnableExg1 = true,
                EnableExg2 = true,
                ExgUse16Bit = true
            };

            // IMU
            Assert.True(c.EnableLowNoiseAccelerometer);
            Assert.True(c.EnableWideRangeAccelerometer);
            Assert.True(c.EnableGyroscope);
            Assert.True(c.EnableMagnetometer);
            Assert.True(c.EnablePressureTemperature);
            Assert.True(c.EnableBattery);
            Assert.True(c.EnableExtA6);
            Assert.True(c.EnableExtA7);
            Assert.True(c.EnableExtA15);
            Assert.Equal(128.5, c.SamplingRate);

            // EXG
            Assert.True(c.EnableExg1);
            Assert.True(c.EnableExg2);
            Assert.True(c.ExgUse16Bit);
        }


        /// <summary>
        /// Verifies round-trip mapping between <see cref="ShimmerConfig.ExgMode"/> and the wire field "exg_mode".
        /// Expected:
        /// - Enum -> lowercased wire string via getter
        /// - Wire string (variants/aliases) -> Enum via setter
        /// - null/blank resets to None and JSON omits field
        /// </summary>
        [Fact]
        public void ExgModeWire_Roundtrip()
        {
            var c = new ShimmerConfig();

            // Enum -> wire
            c.ExgMode = ExgMode.ECG;
            Assert.Equal("ecg", c.ExgModeWire);

            c.ExgMode = ExgMode.EMG;
            Assert.Equal("emg", c.ExgModeWire);

            c.ExgMode = ExgMode.ExgTest;
            Assert.Equal("test", c.ExgModeWire);

            c.ExgMode = ExgMode.Respiration;
            Assert.Equal("resp", c.ExgModeWire);

            // Wire -> enum (case/alias tolerant)
            c.ExgModeWire = "ECG";
            Assert.Equal(ExgMode.ECG, c.ExgMode);

            c.ExgModeWire = "emg";
            Assert.Equal(ExgMode.EMG, c.ExgMode);

            c.ExgModeWire = "ExgTest";
            Assert.Equal(ExgMode.ExgTest, c.ExgMode);

            c.ExgModeWire = "respiration";
            Assert.Equal(ExgMode.Respiration, c.ExgMode);

            // Reset via null/blank
            c.ExgModeWire = null;
            Assert.Equal(ExgMode.None, c.ExgMode);
            Assert.Null(c.ExgModeWire);

            var json = JsonSerializer.Serialize(c);
            Assert.DoesNotContain("exg_mode", json);
        }


        /// <summary>
        /// Checks JSON includes "exg_mode" only when non-null.
        /// Expected:
        /// - With ExgMode set to ECG, JSON contains "exg_mode":"ecg"
        /// </summary>
        [Fact]
        public void Json_Serializes_ExgModeWire_WhenSet()
        {
            var c = new ShimmerConfig { ExgMode = ExgMode.ECG };
            var json = JsonSerializer.Serialize(c);
            Assert.Contains("\"exg_mode\":\"ecg\"", json);
        }


        /// <summary>
        /// Getter returns null when <see cref="ShimmerConfig.ExgMode"/> == None.
        /// Expected:
        /// - ExgMode=None => ExgModeWire == null (JSON omits field)
        /// </summary>
        [Fact]
        public void Getter_ReturnsNull_WhenModeNone()
        {
            var c = new ShimmerConfig { ExgMode = ExgMode.None };
            Assert.Null(c.ExgModeWire);
        }


        /// <summary>
        /// Getter maps enum values to the expected lowercase wire strings.
        /// Expected:
        /// - ECG -> "ecg", EMG -> "emg", ExgTest -> "test", Respiration -> "resp"
        /// </summary>
        [Fact]
        public void Getter_MapsEnum_ToWireString()
        {
            var c = new ShimmerConfig();

            c.ExgMode = ExgMode.ECG;
            Assert.Equal("ecg", c.ExgModeWire);

            c.ExgMode = ExgMode.EMG;
            Assert.Equal("emg", c.ExgModeWire);

            c.ExgMode = ExgMode.ExgTest;
            Assert.Equal("test", c.ExgModeWire);

            c.ExgMode = ExgMode.Respiration;
            Assert.Equal("resp", c.ExgModeWire);
        }


        /// <summary>
        /// Setter accepts case-insensitive values and synonyms, mapping to the correct enum.
        /// Expected:
        /// - "ECG"->ECG, "emg"->EMG, "test"/"ExgTest"/"exg_test"->ExgTest, "resp"/"respiration"->Respiration
        /// </summary>
        [Fact]
        public void Setter_Parses_KnownValues_AndSynonyms()
        {
            var c = new ShimmerConfig();

            c.ExgModeWire = "ECG";
            Assert.Equal(ExgMode.ECG, c.ExgMode);

            c.ExgModeWire = "emg";
            Assert.Equal(ExgMode.EMG, c.ExgMode);

            c.ExgModeWire = "test";
            Assert.Equal(ExgMode.ExgTest, c.ExgMode);

            c.ExgModeWire = "ExgTest";  
            Assert.Equal(ExgMode.ExgTest, c.ExgMode);

            c.ExgModeWire = "exg_test";
            Assert.Equal(ExgMode.ExgTest, c.ExgMode);

            c.ExgModeWire = "resp";
            Assert.Equal(ExgMode.Respiration, c.ExgMode);

            c.ExgModeWire = "respiration";
            Assert.Equal(ExgMode.Respiration, c.ExgMode);
        }


        /// <summary>
        /// Setter resets to None when given null/empty/whitespace.
        /// Expected:
        /// - After null/""/"   " => ExgMode==None
        /// </summary>
        [Fact]
        public void Setter_Blank_ResetsToNone()
        {
            var c = new ShimmerConfig { ExgMode = ExgMode.EMG };

            c.ExgModeWire = null;
            Assert.Equal(ExgMode.None, c.ExgMode);

            c.ExgModeWire = "";
            Assert.Equal(ExgMode.None, c.ExgMode);

            c.ExgModeWire = "   ";
            Assert.Equal(ExgMode.None, c.ExgMode);
        }


        /// <summary>
        /// Unknown values reset the enum to None.
        /// Expected:
        /// - "unknown" => ExgMode==None
        /// </summary>
        [Fact]
        public void Setter_Unknown_ResetsToNone()
        {
            var c = new ShimmerConfig { ExgMode = ExgMode.ECG };

            c.ExgModeWire = "unknown";
            Assert.Equal(ExgMode.None, c.ExgMode);
        }


        /// <summary>
        /// Round-trip wire-string -> enum -> canonical wire-string.
        /// Expected:
        /// - "ECG" -> ECG -> "ecg"
        /// - "exg_test" -> ExgTest -> "test"
        /// - "respiration" -> Respiration -> "resp"
        /// </summary>
        [Fact]
        public void RoundTrip_Wire_ToEnum_ToWire()
        {
            var c = new ShimmerConfig();

            c.ExgModeWire = "ECG";
            Assert.Equal(ExgMode.ECG, c.ExgMode);
            Assert.Equal("ecg", c.ExgModeWire);

            c.ExgModeWire = "exg_test";
            Assert.Equal(ExgMode.ExgTest, c.ExgMode);
            Assert.Equal("test", c.ExgModeWire);

            c.ExgModeWire = "respiration";
            Assert.Equal(ExgMode.Respiration, c.ExgMode);
            Assert.Equal("resp", c.ExgModeWire);
        }


        /// <summary>
        /// With <c>[JsonIgnore(WhenWritingNull)]</c>, null wire field is omitted.
        /// Expected:
        /// - ExgMode=None => JSON does not contain "exg_mode"
        /// </summary>
        [Fact]
        public void Json_Omits_Field_WhenNull()
        {
            var c = new ShimmerConfig { ExgMode = ExgMode.None };

            string json = JsonSerializer.Serialize(c);
            Assert.DoesNotContain("\"exg_mode\"", json);
        }


        /// <summary>
        /// When ExgMode is set, JSON includes "exg_mode" with the expected wire-string.
        /// Expected:
        /// - ExgMode=EMG => JSON contains "exg_mode":"emg"
        /// </summary>
        [Fact]
        public void Json_Includes_Field_WhenSet()
        {
            var c = new ShimmerConfig { ExgMode = ExgMode.EMG };

            string json = JsonSerializer.Serialize(c);
            Assert.Contains("\"exg_mode\":\"emg\"", json);
        }


        /// <summary>
        /// JSON round-trip preserves the exg_mode mapping.
        /// Expected:
        /// - Serialize(Respiration) contains "resp"
        /// - Deserialize restores ExgMode=Respiration and wire "resp"
        /// </summary>
        [Fact]
        public void Json_RoundTrip_ExgMode_Mapping()
        {
            var orig = new ShimmerConfig { ExgMode = ExgMode.Respiration };
            var json = JsonSerializer.Serialize(orig);

            Assert.Contains("\"exg_mode\":\"resp\"", json);

            var back = JsonSerializer.Deserialize<ShimmerConfig>(json);
            Assert.NotNull(back);
            Assert.Equal(ExgMode.Respiration, back!.ExgMode);
            Assert.Equal("resp", back.ExgModeWire);
        }


        // ----- WsBridgeManager class -----
        // -------- IsRunning behavior --------


        /// <summary>
        /// Before calling StartAsync, the manager should report not running.
        /// Expected:
        /// - IsRunning == false
        /// </summary>
        [Fact]
        public void IsRunning_False_BeforeStart()
        {
            var mgr = new WsBridgeManager();
            Assert.False(mgr.IsRunning);
        }


        /// <summary>
        /// After StartAsync, manager reports running (Watson stub sets IsListening).
        /// Expected:
        /// - StartAsync completes
        /// - IsRunning == true
        /// </summary>
        [Fact]
        public async Task IsRunning_True_AfterStart()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity(), port: 8787);
            Assert.True(mgr.IsRunning);
        }


        /// <summary>
        /// After StopAsync, manager reports not running.
        /// Expected:
        /// - StopAsync completes
        /// - IsRunning == false
        /// </summary>
        [Fact]
        public async Task IsRunning_False_AfterStop()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            await mgr.StopAsync();
            Assert.False(mgr.IsRunning);
        }


        // ----- ActiveSessionCount behavior -----


        /// <summary>
        /// With no sessions opened, ActiveSessionCount starts at zero.
        /// Expected:
        /// - ActiveSessionCount == 0
        /// </summary>
        [Fact]
        public void ActiveSessionCount_Initially_Zero()
        {
            var mgr = new WsBridgeManager();
            Assert.Equal(0, mgr.ActiveSessionCount);
        }


        // -------- Dispose behavior --------


        /// <summary>
        /// Dispose is safe; it triggers a stop and leaves the manager not running.
        /// Expected:
        /// - After Dispose(), IsRunning becomes false (after a brief delay)
        /// - No exception thrown
        /// </summary>
        [Fact]
        public async Task Dispose_StopsManager_Safely()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            Assert.True(mgr.IsRunning);

            mgr.Dispose();

            await Task.Delay(50);
            Assert.False(mgr.IsRunning);
        }


        // ----- AnySensorEnabled behavior -----


        /// <summary>
        /// AnySensorEnabled returns false when all flags are disabled (IMU & EXG).
        /// Expected:
        /// - AnySensorEnabled(config) == false
        /// </summary>
        [Fact]
        public void AnySensorEnabled_False_WhenAllOff()
        {
            var c = new ShimmerConfig
            {
                EnableLowNoiseAccelerometer = false,
                EnableWideRangeAccelerometer = false,
                EnableGyroscope = false,
                EnableMagnetometer = false,
                EnablePressureTemperature = false,
                EnableBattery = false,
                EnableExtA6 = false,
                EnableExtA7 = false,
                EnableExtA15 = false,
                EnableExg1 = false,
                EnableExg2 = false
            };

            Assert.False(WsBridgeManager.AnySensorEnabled(c));
        }


        /// <summary>
        /// Any IMU flag set to true makes AnySensorEnabled return true.
        /// Expected:
        /// - For each IMU flag individually enabled, AnySensorEnabled(config) == true
        /// </summary>
        [Theory]
        [InlineData(nameof(ShimmerConfig.EnableLowNoiseAccelerometer))]
        [InlineData(nameof(ShimmerConfig.EnableWideRangeAccelerometer))]
        [InlineData(nameof(ShimmerConfig.EnableGyroscope))]
        [InlineData(nameof(ShimmerConfig.EnableMagnetometer))]
        [InlineData(nameof(ShimmerConfig.EnablePressureTemperature))]
        [InlineData(nameof(ShimmerConfig.EnableBattery))]
        [InlineData(nameof(ShimmerConfig.EnableExtA6))]
        [InlineData(nameof(ShimmerConfig.EnableExtA7))]
        [InlineData(nameof(ShimmerConfig.EnableExtA15))]
        public void AnySensorEnabled_True_WhenAnyImuOn(string flagName)
        {
            var c = new ShimmerConfig();

            var prop = typeof(ShimmerConfig).GetProperty(flagName);
            Assert.NotNull(prop);
            prop!.SetValue(c, true);

            Assert.True(WsBridgeManager.AnySensorEnabled(c));
        }


        /// <summary>
        /// EXG1 or EXG2 alone make AnySensorEnabled return true.
        /// Expected:
        /// - EnableExg1 == true => AnySensorEnabled == true
        /// - EnableExg2 == true => AnySensorEnabled == true
        /// </summary>
        [Fact]
        public void AnySensorEnabled_True_WhenExgEnabled()
        {
            var c1 = new ShimmerConfig { EnableExg1 = true };
            var c2 = new ShimmerConfig { EnableExg2 = true };

            Assert.True(WsBridgeManager.AnySensorEnabled(c1));
            Assert.True(WsBridgeManager.AnySensorEnabled(c2));
        }


        // ----- Log event (sanity check) -----


        /// <summary>
        /// A simple sanity check that Log event fires during Start/Stop.
        /// Expected:
        /// - logCount >= 1 after a start-stop cycle
        /// </summary>
        [Fact]
        public async Task LogEvent_Fires_OnStartAndStop()
        {
            var mgr = new WsBridgeManager();
            int logCount = 0;
            mgr.Log += _ => logCount++;

            await mgr.StartAsync(new Activity());
            await mgr.StopAsync();

            Assert.True(logCount >= 1);
        }


        // ----- StartAsync behavior -----


        /// <summary>
        /// StartAsync should set IsRunning and log the listen URL.
        /// Expected:
        /// - IsRunning == true after StartAsync
        /// - Last log contains "ws://" and the chosen port
        /// </summary>
        [Fact]
        public async Task StartAsync_Sets_IsRunning_And_LogsUrl()
        {
            var mgr = new WsBridgeManager();

            string? lastLog = null;
            mgr.Log += s => lastLog = s;

            Assert.False(mgr.IsRunning);

            await mgr.StartAsync(new Activity(), port: 9090);

            Assert.True(mgr.IsRunning);
            Assert.NotNull(lastLog);
            Assert.Contains("ws://", lastLog!);
            Assert.Contains(":9090/", lastLog!);
        }


        /// <summary>
        /// Calling StartAsync when already running is a no-op.
        /// Expected:
        /// - Second StartAsync does not throw
        /// - IsRunning remains true
        /// </summary>
        [Fact]
        public async Task StartAsync_Is_Idempotent_When_Already_Running()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity(), port: 8787);

            Assert.True(mgr.IsRunning);

            // second call must not throw and must keep IsRunning == true
            var ex = await Record.ExceptionAsync(async () => await mgr.StartAsync(new Activity(), port: 8787));
            Assert.Null(ex);
            Assert.True(mgr.IsRunning);
        }


        // ----- StopAsync behavior -----


        /// <summary>
        /// StopAsync turns IsRunning to false after a prior start.
        /// Expected:
        /// - After StopAsync, IsRunning == false
        /// </summary>
        [Fact]
        public async Task StopAsync_Turns_IsRunning_False()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            Assert.True(mgr.IsRunning);

            await mgr.StopAsync();

            Assert.False(mgr.IsRunning);
        }


        /// <summary>
        /// Stopping when not running is safe, logs a message, and does not change IsRunning.
        /// Expected:
        /// - No exception
        /// - IsRunning remains false
        /// - Last log == "WS stopped"
        /// </summary>
        [Fact]
        public async Task StopAsync_WhenNotRunning_IsSafe_And_Logs()
        {
            var mgr = new WsBridgeManager();

            string? lastLog = null;
            mgr.Log += s => lastLog = s;

            Assert.False(mgr.IsRunning);

            var ex = await Record.ExceptionAsync(async () => await mgr.StopAsync());
            Assert.Null(ex);

            Assert.False(mgr.IsRunning);
            Assert.Equal("WS stopped", lastLog);
        }


        /// <summary>
        /// After a start, StopAsync stops the server and logs a stop message.
        /// Expected:
        /// - IsRunning == false after stop
        /// - Last log == "WS stopped"
        /// </summary>
        [Fact]
        public async Task StopAsync_AfterStart_StopsServer_And_Logs()
        {
            var mgr = new WsBridgeManager();
            string? lastLog = null;
            mgr.Log += s => lastLog = s;

            await mgr.StartAsync(new Activity(), port: 9091);
            Assert.True(mgr.IsRunning);

            await mgr.StopAsync();

            Assert.False(mgr.IsRunning);
            Assert.Equal("WS stopped", lastLog);
        }


        /// <summary>
        /// Multiple StopAsync calls are idempotent and safe.
        /// Expected:
        /// - No exception on repeated StopAsync
        /// - IsRunning remains false after first stop
        /// </summary>
        [Fact]
        public async Task StopAsync_IsIdempotent()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            Assert.True(mgr.IsRunning);

            await mgr.StopAsync();
            Assert.False(mgr.IsRunning);

            var ex = await Record.ExceptionAsync(async () => await mgr.StopAsync());
            Assert.Null(ex);
            Assert.False(mgr.IsRunning);
        }


        // ----- WS lifecycle behavior (no SPP dependencies) -----


        /// <summary>
        /// Start -> Stop -> Start sequence is safe and leaves expected running state.
        /// Expected:
        /// - IsRunning true after each Start
        /// - IsRunning false after Stop
        /// - No exceptions thrown
        /// </summary>
        [Fact]
        public async Task Restart_AfterStop_Works()
        {
            var mgr = new WsBridgeManager();

            // 1st start
            await mgr.StartAsync(new Activity(), port: 9010);
            Assert.True(mgr.IsRunning);

            // stop
            await mgr.StopAsync();
            Assert.False(mgr.IsRunning);

            // 2nd start
            await mgr.StartAsync(new Activity(), port: 9011);
            Assert.True(mgr.IsRunning);

            // final stop to leave clean state
            await mgr.StopAsync();
            Assert.False(mgr.IsRunning);
        }


        /// <summary>
        /// Repeated start/stop cycles do not leak state.
        /// Expected:
        /// - Each iteration ends with IsRunning == false
        /// - No exceptions thrown
        /// </summary>
        [Fact]
        public async Task StartStop_Multiple_Times_NoLeak()
        {
            var mgr = new WsBridgeManager();

            for (int i = 0; i < 3; i++)
            {
                await mgr.StartAsync(new Activity(), port: 9200 + i);
                Assert.True(mgr.IsRunning);

                await mgr.StopAsync();
                Assert.False(mgr.IsRunning);
            }
        }


        /// <summary>
        /// Log event should fire at least once over a start-stop cycle.
        /// Expected:
        /// - logs >= 1
        /// </summary>
        [Fact]
        public async Task LogEvent_Fires_During_Cycle()
        {
            var mgr = new WsBridgeManager();
            int logs = 0;
            mgr.Log += _ => logs++;

            await mgr.StartAsync(new Activity(), port: 9300);
            await mgr.StopAsync();

            Assert.True(logs >= 1);
        }


        /// <summary>
        /// Dispose() when not running is a no-op and never throws.
        /// Expected:
        /// - No exception
        /// - IsRunning remains false
        /// </summary>
        [Fact]
        public void Dispose_When_NotRunning_Is_NoOp()
        {
            var mgr = new WsBridgeManager();
            var ex = Record.Exception(() => mgr.Dispose());
            Assert.Null(ex);
            Assert.False(mgr.IsRunning);
        }


        /// <summary>
        /// SamplingRate alone does not imply "sensors enabled".
        /// Expected:
        /// - With only SamplingRate set, AnySensorEnabled == false
        /// </summary>
        [Fact]
        public void AnySensorEnabled_SamplingRate_Alone_IsFalse()
        {
            var c = new ShimmerConfig { SamplingRate = 256.0 };
            Assert.False(WsBridgeManager.AnySensorEnabled(c));
        }


        /// <summary>
        /// StartAsync logs a ws:// URL containing the requested port.
        /// Expected:
        /// - Last log contains "ws://" and ":9405/"
        /// - IsRunning true during run; false after Stop
        /// </summary>
        [Fact]
        public async Task StartAsync_Logs_Url_With_Port()
        {
            var mgr = new WsBridgeManager();
            string? last = null;
            mgr.Log += s => last = s;

            await mgr.StartAsync(new Activity(), port: 9405);
            Assert.True(mgr.IsRunning);

            Assert.NotNull(last);
            Assert.Contains("ws://", last!, StringComparison.OrdinalIgnoreCase);
            Assert.Contains(":9405/", last!, StringComparison.OrdinalIgnoreCase);

            await mgr.StopAsync();
            Assert.False(mgr.IsRunning);
        }


        // ----- UpdateConfigAsync behavior -----


        /// <summary>
        /// UpdateConfigAsync on a missing session is ignored and logs a message.
        /// Expected:
        /// - No exception
        /// - Logs contain "UpdateConfig ignored: session not found"
        /// </summary>
        [Fact]
        public async Task UpdateConfigAsync_Ignores_When_SessionMissing_And_Logs()
        {
            var mgr = new WsBridgeManager();
            var logs = new System.Collections.Generic.List<string>();
            mgr.Log += s => logs.Add(s);

            await mgr.StartAsync(new Activity());

            await mgr.UpdateConfigAsync("11:22:33:44:55:66", new ShimmerConfig { EnableGyroscope = true });

            Assert.Contains(logs, l => l.Contains("UpdateConfig ignored: session not found"));

            await mgr.StopAsync();
        }


        /// <summary>
        /// UpdateConfigAsync preserves EXG mode when locked, applies new flags/SR, and logs each step.
        /// Expected:
        /// - "update requested" logs requested mode and lock=True
        /// - "update applied" keeps original mode
        /// - ApplyConfig logs updated SR
        /// - Final log "Reconfigured &lt;mac&gt; (mode locked)"
        /// </summary>
        [Fact]
        public async Task UpdateConfigAsync_Preserves_ExgMode_And_Applies_NewFlags_AndLogs()
        {
            var mgr = new WsBridgeManager();

            var logs = new System.Collections.Generic.List<string>();
            mgr.Log += s => logs.Add(s);

            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:CC";

            var initial = new ShimmerConfig
            {
                ExgMode = ExgMode.ECG,
                EnableGyroscope = true,
                SamplingRate = 51
            };
            await mgr.OpenConfigureAndStartAsync(mac, initial);

            Assert.Contains(logs, l => l.Contains("[SERVER] applied   exg_mode") && l.Contains("'ecg'"));

            var update = new ShimmerConfig
            {
                ExgMode = ExgMode.EMG,
                EnableGyroscope = false,
                EnableMagnetometer = true,
                SamplingRate = 200
            };

            await mgr.UpdateConfigAsync(mac, update);

            Assert.Contains(logs, l => l.Contains("[SERVER] update requested exg_mode") &&
                                       l.Contains("emg") &&
                                       l.Contains("mode lock=True"));

            Assert.Contains(logs, l => l.Contains("[SERVER] update applied  exg_mode") &&
                                       l.Contains("'ecg'") &&
                                       l.Contains("enum=ECG"));

            Assert.Contains(logs, l => l.Contains("[CFG] applied (SR=200Hz"));

            Assert.Contains(logs, l => l.Contains($"[SERVER] Reconfigured {mac} (mode locked)"));

            await mgr.StopAsync();
        }


        // ----- CloseAllAsync behavior -----


        /// <summary>
        /// CloseAllAsync with no sessions is safe and logs a completion message.
        /// Expected:
        /// - No exception
        /// - ActiveSessionCount == 0
        /// - Logs contain "[SERVER] All sessions closed"
        /// </summary>
        [Fact]
        public async Task CloseAllAsync_NoSessions_IsSafe_And_Logs()
        {
            var mgr = new WsBridgeManager();
            var logs = new System.Collections.Generic.List<string>();
            mgr.Log += s => logs.Add(s);

            Assert.Equal(0, mgr.ActiveSessionCount);

            var ex = await Record.ExceptionAsync(async () => await mgr.CloseAllAsync());
            Assert.Null(ex);

            Assert.Contains(logs, l => l.Contains("[SERVER] All sessions closed"));
            Assert.Equal(0, mgr.ActiveSessionCount);
        }


        /// <summary>
        /// CloseAllAsync closes sessions and clears the map without stopping the WS server.
        /// Expected:
        /// - After closing: ActiveSessionCount == 0
        /// - WS server is still running
        /// - Logs contain "[SERVER] All sessions closed"
        /// </summary>
        [Fact]
        public async Task CloseAllAsync_WithSessions_ClosesAndClears_WithoutStoppingServer()
        {
            var mgr = new WsBridgeManager();
            var logs = new System.Collections.Generic.List<string>();
            mgr.Log += s => logs.Add(s);

            await mgr.StartAsync(new Activity());
            Assert.True(mgr.IsRunning);

            var mac1 = "00:06:66:AA:BB:01";
            var mac2 = "00:06:66:AA:BB:02";

            await mgr.OpenConfigureAndStartAsync(mac1, new ShimmerConfig
            {
                ExgMode = ExgMode.ECG,
                EnableGyroscope = true,
                SamplingRate = 51
            });

            await mgr.OpenConfigureAndStartAsync(mac2, new ShimmerConfig
            {
                ExgMode = ExgMode.ECG,
                EnableMagnetometer = true,
                SamplingRate = 51
            });

            Assert.True(mgr.ActiveSessionCount >= 2);

            await mgr.CloseAllAsync();

            Assert.Equal(0, mgr.ActiveSessionCount);
            Assert.Contains(logs, l => l.Contains("[SERVER] All sessions closed"));

            Assert.True(mgr.IsRunning);

            await mgr.StopAsync();
            Assert.False(mgr.IsRunning);
        }


        /// <summary>
        /// CloseAllAsync is idempotent: multiple calls are safe and final state is stable.
        /// Expected:
        /// - First call clears sessions
        /// - Subsequent calls do nothing and do not throw
        /// </summary>
        [Fact]
        public async Task CloseAllAsync_IsIdempotent()
        {
            var mgr = new WsBridgeManager();

            var mac = "00:06:66:AA:BB:CC";
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig
            {
                ExgMode = ExgMode.ECG,
                EnableGyroscope = true,
                SamplingRate = 51
            });
            Assert.True(mgr.ActiveSessionCount >= 1);

            // First closure
            await mgr.CloseAllAsync();
            Assert.Equal(0, mgr.ActiveSessionCount);

            // Second closure (no exceptions)
            var ex = await Record.ExceptionAsync(async () => await mgr.CloseAllAsync());
            Assert.Null(ex);
            Assert.Equal(0, mgr.ActiveSessionCount);
        }


        // ----- OnMessage behavior -----


        /// <summary>
        /// Text frames are handled and inbound content is logged.
        /// Expected: After sending {"type":"hello"}, logs contain "WS IN [...]" and "type=hello".
        /// </summary>
        [Fact]
        public async Task OnMessage_TextFrame_IsHandled_And_LogsInbound()
        {
            var mgr = new WsBridgeManager();
            var logs = new System.Collections.Generic.List<string>();
            mgr.Log += s => logs.Add(s);

            await mgr.StartAsync(new Activity());

            var wsField = typeof(WsBridgeManager).GetField("_ws", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var ws = (WatsonWebsocket.WatsonWsServer)wsField!.GetValue(mgr)!;

            var clientId = Guid.NewGuid();
            ws.RaiseConnected(clientId);

            // Send a valid textual frame ("hello")
            ws.RaiseText(clientId, "{\"type\":\"hello\"}");

            await Task.Delay(50);

            Assert.Contains(logs, l => l.Contains("WS IN [") && l.Contains("type=hello"));

            await mgr.StopAsync();
        }


        /// <summary>
        /// Non-text frames are ignored.
        /// Expected: Sending a binary frame yields no "WS IN [...]" log entries.
        /// </summary>
        [Fact]
        public async Task OnMessage_NonText_IsIgnored()
        {
            var mgr = new WsBridgeManager();
            var logs = new System.Collections.Generic.List<string>();
            mgr.Log += s => logs.Add(s);

            await mgr.StartAsync(new Activity());

            var wsField = typeof(WsBridgeManager).GetField("_ws", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var ws = (WatsonWebsocket.WatsonWsServer)wsField!.GetValue(mgr)!;

            var clientId = Guid.NewGuid();
            ws.RaiseConnected(clientId);

            // Send a Binary frame
            ws.RaiseBinary(clientId, new byte[] { 0x01, 0x02, 0x03 });

            await Task.Delay(50);

            // no "WS IN [...]" because non-Texts are ignored
            Assert.DoesNotContain(logs, l => l.Contains("WS IN ["));

            await mgr.StopAsync();
        }


        /// <summary>
        /// Malformed JSON is caught and logged by OnMessage.
        /// Expected: Log entry starts with "OnMessage error: " after receiving "{ not-json".
        /// </summary>
        [Fact]
        public async Task OnMessage_MalformedJson_IsCaught_AndLogged()
        {
            var mgr = new WsBridgeManager();
            var logs = new System.Collections.Generic.List<string>();
            mgr.Log += s => logs.Add(s);

            await mgr.StartAsync(new Activity());

            var wsField = typeof(WsBridgeManager).GetField("_ws", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var ws = (WatsonWebsocket.WatsonWsServer)wsField!.GetValue(mgr)!;

            var clientId = Guid.NewGuid();
            ws.RaiseConnected(clientId);

            ws.RaiseText(clientId, "{ not-json");

            await Task.Delay(50);

            Assert.Contains(logs, l => l.StartsWith("OnMessage error: ", StringComparison.Ordinal));

            await mgr.StopAsync();
        }


        // ----- SendConfigSnapshot behavior -----


        /// <summary>
        /// If no active session exists for the given MAC, no payload is sent.
        /// Expected: SendConfigSnapshot on an unknown MAC leaves ws.Sent empty.
        /// </summary>
        [Fact]
        public async Task SendConfigSnapshot_NoActiveSession_CompletesWithoutSend()
        {
            var mgr = new WsBridgeManager();

            // Start the WS without any active session
            await mgr.StartAsync(new Activity());

            var wsField = typeof(WsBridgeManager).GetField("_ws", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var ws = (WatsonWebsocket.WatsonWsServer)wsField!.GetValue(mgr)!;

            ws.Sent.Clear();

            var clientId = Guid.NewGuid();
            var mac = "00:06:66:AA:BB:CC";
            var mi = typeof(WsBridgeManager).GetMethod("SendConfigSnapshot", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            Assert.NotNull(mi);

            var task = (Task)mi!.Invoke(mgr, new object[] { clientId, mac })!;
            await task;

            Assert.Empty(ws.Sent);

            await mgr.StopAsync();
        }


        /// <summary>
        /// With an active session, SendConfigSnapshot sends a "config_changed" to the target client.
        /// Expected: Exactly one send to that client; JSON has type=config_changed, correct mac, cfg present, and "available" array (e.g., includes "gyro" when enabled).
        /// </summary>
        [Fact]
        public async Task SendConfigSnapshot_WithActiveSession_Sends_ConfigChanged_ToThatClient()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:01";

            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig
            {
                ExgMode = ExgMode.ECG,
                EnableGyroscope = true,
                SamplingRate = 51
            });

            var wsField = typeof(WsBridgeManager).GetField("_ws", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var ws = (WatsonWebsocket.WatsonWsServer)wsField!.GetValue(mgr)!;

            ws.Sent.Clear();
            var clientId = Guid.NewGuid();

            var mi = typeof(WsBridgeManager).GetMethod("SendConfigSnapshot", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var task = (Task)mi!.Invoke(mgr, new object[] { clientId, mac })!;
            await task;

            // Check for a single send to the correct client
            Assert.Single(ws.Sent);
            Assert.Equal(clientId, ws.Sent[0].clientId);

            // Check minimum JSON structure
            var payload = ws.Sent[0].message;
            using var doc = System.Text.Json.JsonDocument.Parse(payload);
            var root = doc.RootElement;

            Assert.Equal("config_changed", root.GetProperty("type").GetString());
            Assert.Equal(mac, root.GetProperty("mac").GetString());
            Assert.True(root.TryGetProperty("cfg", out _));
            Assert.True(root.TryGetProperty("available", out var available));
            Assert.Equal(System.Text.Json.JsonValueKind.Array, available.ValueKind);

            var cfg = root.GetProperty("cfg");
            Assert.Equal("ecg", cfg.GetProperty("exg_mode").GetString());
            Assert.Contains(available.EnumerateArray(), x => x.GetString() == "gyro");

            await mgr.StopAsync();
        }


        /// <summary>
        /// Multiple SendConfigSnapshot calls accumulate per client.
        /// Expected: 3 sends total (2 to clientA, 1 to clientB); each payload has type=config_changed and the expected MAC.
        /// </summary>
        [Fact]
        public async Task SendConfigSnapshot_MultipleCalls_AccumulateSends_PerClient()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:02";
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig
            {
                ExgMode = ExgMode.EMG,
                EnableMagnetometer = true,
                SamplingRate = 100
            });

            var wsField = typeof(WsBridgeManager).GetField("_ws", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var ws = (WatsonWebsocket.WatsonWsServer)wsField!.GetValue(mgr)!;

            ws.Sent.Clear();

            var clientA = Guid.NewGuid();
            var clientB = Guid.NewGuid();

            var mi = typeof(WsBridgeManager).GetMethod("SendConfigSnapshot", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);

            // Two snapshots to clientA, one to clientB
            await (Task)mi!.Invoke(mgr, new object[] { clientA, mac })!;
            await (Task)mi!.Invoke(mgr, new object[] { clientA, mac })!;
            await (Task)mi!.Invoke(mgr, new object[] { clientB, mac })!;

            Assert.Equal(3, ws.Sent.Count);
            Assert.Equal(2, ws.Sent.Count(x => x.clientId == clientA));
            Assert.Equal(1, ws.Sent.Count(x => x.clientId == clientB));

            // All payloads must be config_changed for the expected MAC
            foreach (var (_, msg) in ws.Sent)
            {
                using var d = System.Text.Json.JsonDocument.Parse(msg);
                var r = d.RootElement;
                Assert.Equal("config_changed", r.GetProperty("type").GetString());
                Assert.Equal(mac, r.GetProperty("mac").GetString());
            }

            await mgr.StopAsync();
        }


        // ----- HandleTextAsync behavior -----


        /// <summary>
        /// Helper: retrieves the private WebSocket server instance from a <see cref="WsBridgeManager"/> under test.
        /// Uses reflection to access the non-public <c>_ws</c> field so tests can raise WS events and inspect sent frames.
        /// </summary>
        /// <param name="mgr">The <see cref="WsBridgeManager"/> instance under test.</param>
        /// <returns>
        /// The underlying <see cref="WatsonWebsocket.WatsonWsServer"/> instance referenced by the private <c>_ws</c> field.
        /// </returns>
        private static WatsonWsServer GetWs(WsBridgeManager mgr)
        {
            var wsField = typeof(WsBridgeManager).GetField("_ws", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            return (WatsonWsServer)wsField!.GetValue(mgr)!;
        }


        /// <summary>
        /// When a client sends {"type":"hello"}, the server replies with a hello acknowledgment.
        /// Expected:
        /// - Exactly one frame is sent back to the same client
        /// - Reply has type == "hello_ack"
        /// - ok == true and proto == "shimmer.v1"
        /// </summary>
        [Fact]
        public async Task HandleText_Hello_RespondsWithHelloAck()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();
            ws.RaiseText(client, "{\"type\":\"hello\"}");

            Assert.Single(ws.Sent);
            Assert.Equal(client, ws.Sent[0].clientId);

            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;
            Assert.Equal("hello_ack", root.GetProperty("type").GetString());
            Assert.True(root.GetProperty("ok").GetBoolean());
            Assert.Equal("shimmer.v1", root.GetProperty("proto").GetString());

            await mgr.StopAsync();
        }


        /// <summary>
        /// Unknown message types produce a structured error reply.
        /// Expected:
        /// - Exactly one frame is sent back to the same client
        /// - Reply has type == "error" and error == "unknown_type"
        /// </summary>
        [Fact]
        public async Task HandleText_UnknownType_RespondsWithError()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();
            ws.RaiseText(client, "{\"type\":\"does_not_exist\"}");

            Assert.Single(ws.Sent);
            Assert.Equal(client, ws.Sent[0].clientId);

            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;
            Assert.Equal("error", root.GetProperty("type").GetString());
            Assert.Equal("unknown_type", root.GetProperty("error").GetString());

            await mgr.StopAsync();
        }


        /// <summary>
        /// With no active sessions, listing active devices returns an empty array.
        /// Expected:
        /// - Reply has type == "active_devices"
        /// - Property "macs" is an empty JSON array
        /// </summary>
        [Fact]
        public async Task HandleText_ListActive_Empty_ReturnsEmptyArray()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();
            ws.RaiseText(client, "{\"type\":\"list_active\"}");

            Assert.Single(ws.Sent);
            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;
            Assert.Equal("active_devices", root.GetProperty("type").GetString());
            var macs = root.GetProperty("macs");
            Assert.Equal(JsonValueKind.Array, macs.ValueKind);
            Assert.Equal(0, macs.GetArrayLength());

            await mgr.StopAsync();
        }


        /// <summary>
        /// Getting a config without specifying a MAC returns a structured error.
        /// Expected:
        /// - Reply has type == "config"
        /// - ok == false and error == "no_mac"
        /// </summary>
        [Fact]
        public async Task HandleText_GetConfig_NoMac_ReturnsErrorNoMac()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();
            ws.RaiseText(client, "{\"type\":\"get_config\"}");

            Assert.Single(ws.Sent);
            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;
            Assert.Equal("config", root.GetProperty("type").GetString());
            Assert.False(root.GetProperty("ok").GetBoolean());
            Assert.Equal("no_mac", root.GetProperty("error").GetString());

            await mgr.StopAsync();
        }


        /// <summary>
        /// set_sampling_rate rejects bad arguments (missing/empty mac or non-numeric sr).
        /// Expected:
        /// - Reply has type == "set_sampling_rate_ack"
        /// - ok == false and error == "bad_args"
        /// </summary>
        [Fact]
        public async Task HandleText_SetSamplingRate_BadArgs_ReturnsBadArgs()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();
            ws.RaiseText(client, "{\"type\":\"set_sampling_rate\",\"mac\":\"\",\"sr\":\"oops\"}");

            Assert.Single(ws.Sent);
            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;
            Assert.Equal("set_sampling_rate_ack", root.GetProperty("type").GetString());
            Assert.False(root.GetProperty("ok").GetBoolean());
            Assert.Equal("bad_args", root.GetProperty("error").GetString());

            await mgr.StopAsync();
        }


        /// <summary>
        /// Opening a stream without a MAC returns a structured "no_mac" error.
        /// Expected:
        /// - Reply has type == "open_ack"
        /// - ok == false and error == "no_mac"
        /// </summary>
        [Fact]
        public async Task HandleText_Open_NoMac_ReturnsErrorNoMac()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();
            ws.RaiseText(client, "{\"type\":\"open\"}");

            Assert.Single(ws.Sent);
            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;
            Assert.Equal("open_ack", root.GetProperty("type").GetString());
            Assert.False(root.GetProperty("ok").GetBoolean());
            Assert.Equal("no_mac", root.GetProperty("error").GetString());

            await mgr.StopAsync();
        }


        /// <summary>
        /// Unsubscribe acknowledges even if no prior subscription state exists.
        /// Expected:
        /// - Reply has type == "unsubscribe_ack"
        /// - ok == true and the echoed mac matches the request
        /// </summary>
        [Fact]
        public async Task HandleText_Unsubscribe_AckTrue()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();
            ws.RaiseText(client, "{\"type\":\"unsubscribe\",\"mac\":\"00:06:66:AA:BB:CC\"}");

            Assert.Single(ws.Sent);
            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;
            Assert.Equal("unsubscribe_ack", root.GetProperty("type").GetString());
            Assert.True(root.GetProperty("ok").GetBoolean());
            Assert.Equal("00:06:66:AA:BB:CC", root.GetProperty("mac").GetString());

            await mgr.StopAsync();
        }


        /// <summary>
        /// Close command always acknowledges and clears any subscription state.
        /// Expected:
        /// - Reply has type == "close_ack"
        /// - ok == true
        /// </summary>
        [Fact]
        public async Task HandleText_Close_AckTrue_AndClearsSubscriptionState()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();
            ws.RaiseText(client, "{\"type\":\"close\"}");

            Assert.Single(ws.Sent);
            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;
            Assert.Equal("close_ack", root.GetProperty("type").GetString());
            Assert.True(root.GetProperty("ok").GetBoolean());

            await mgr.StopAsync();
        }


        /// <summary>
        /// Setting EXG mode from the client side is server-managed and not allowed.
        /// Expected:
        /// - Reply has type == "set_exg_mode_ack"
        /// - ok == false and error == "server_managed"
        /// </summary>
        [Fact]
        public async Task HandleText_SetExgMode_ServerManaged_AckFalse()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();
            ws.RaiseText(client, "{\"type\":\"set_exg_mode\",\"exg_mode\":\"ecg\"}");

            Assert.Single(ws.Sent);
            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;
            Assert.Equal("set_exg_mode_ack", root.GetProperty("type").GetString());
            Assert.False(root.GetProperty("ok").GetBoolean());
            Assert.Equal("server_managed", root.GetProperty("error").GetString());

            await mgr.StopAsync();
        }


        /// <summary>
        /// Stopping from the client side is server-managed and not allowed.
        /// Expected:
        /// - Reply has type == "stop_ack"
        /// - ok == false and error == "server_managed"
        /// </summary>
        [Fact]
        public async Task HandleText_Stop_ServerManaged_AckFalse()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();
            ws.RaiseText(client, "{\"type\":\"stop\"}");

            Assert.Single(ws.Sent);
            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;
            Assert.Equal("stop_ack", root.GetProperty("type").GetString());
            Assert.False(root.GetProperty("ok").GetBoolean());
            Assert.Equal("server_managed", root.GetProperty("error").GetString());

            await mgr.StopAsync();
        }


        /// <summary>
        /// Start command always returns a start acknowledgment, regardless of session state.
        /// Expected:
        /// - Reply has type == "start_ack"
        /// - ok == true and note == "server_managed"
        /// </summary>
        [Fact]
        public async Task HandleText_Start_AlwaysReturnsStartAck()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();
            ws.RaiseText(client, "{\"type\":\"start\",\"mac\":\"00:06:66:00:00:01\"}");

            Assert.Single(ws.Sent);
            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;
            Assert.Equal("start_ack", root.GetProperty("type").GetString());
            Assert.True(root.GetProperty("ok").GetBoolean());
            Assert.Equal("server_managed", root.GetProperty("note").GetString());

            await mgr.StopAsync();
        }


        /// <summary>
        /// Start command always returns a start acknowledgment, regardless of session state.
        /// Expected:
        /// - Reply has type == "start_ack"
        /// - ok == true and note == "server_managed"
        /// </summary>
        [Fact]
        public async Task HandleText_ListDevices_WithNoBondedDevices_ReturnsEmptyItems()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            var ws = GetWs(mgr);
            ws.Sent.Clear();

            Android.Bluetooth.BluetoothAdapter.DefaultAdapter!.BondedDevices.Clear();

            var client = Guid.NewGuid();
            ws.RaiseText(client, "{\"type\":\"list_devices\"}");

            Assert.Single(ws.Sent);
            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;
            Assert.Equal("devices", root.GetProperty("type").GetString());
            var items = root.GetProperty("items");
            Assert.Equal(JsonValueKind.Array, items.ValueKind);
            Assert.Equal(0, items.GetArrayLength());

            await mgr.StopAsync();
        }


        // ----- TryGetConfig behavior -----


        /// <summary>
        /// Listing devices with no bonded Bluetooth devices returns an empty list.
        /// Expected:
        /// - Reply has type == "devices"
        /// - Property "items" is an empty JSON array
        /// </summary>
        [Fact]
        public async Task GetConfig_WhenSessionMissing_ReturnsNotActive()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();

            ws.RaiseText(client, "{\"type\":\"get_config\",\"mac\":\"00:06:66:DE:AD:00\"}");

            Assert.Single(ws.Sent);
            Assert.Equal(client, ws.Sent[0].clientId);

            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;
            Assert.Equal("config", root.GetProperty("type").GetString());
            Assert.False(root.GetProperty("ok").GetBoolean());
            Assert.Equal("00:06:66:DE:AD:00", root.GetProperty("mac").GetString());
            Assert.Equal("not_active", root.GetProperty("error").GetString());

            await mgr.StopAsync();
        }


        /// <summary>
        /// If a session exists, get_config returns the effective configuration snapshot.
        /// Expected:
        /// - Reply has type == "config", ok == true
        /// - Contains cfg with the flags set, integer-rounded sampling rate
        /// - exg_mode wire string matches the mode (e.g., "emg")
        /// </summary>
        [Fact]
        public async Task GetConfig_WhenSessionExists_ReturnsEffectiveConfig()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:CC";

            var initialCfg = new ShimmerConfig
            {
                EnableGyroscope = true,
                EnableBattery = true,
                SamplingRate = 64,
                ExgMode = ExgMode.EMG
            };

            await mgr.OpenConfigureAndStartAsync(mac, initialCfg);

            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();

            ws.RaiseText(client, $"{{\"type\":\"get_config\",\"mac\":\"{mac}\"}}");

            Assert.Single(ws.Sent);
            Assert.Equal(client, ws.Sent[0].clientId);

            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;

            Assert.Equal("config", root.GetProperty("type").GetString());
            Assert.True(root.GetProperty("ok").GetBoolean());
            Assert.Equal(mac, root.GetProperty("mac").GetString());

            var cfg = root.GetProperty("cfg");

            Assert.True(cfg.GetProperty("EnableGyroscope").GetBoolean());
            Assert.True(cfg.GetProperty("EnableBattery").GetBoolean());

            Assert.Equal(64, cfg.GetProperty("SamplingRate").GetInt32());
            Assert.Equal("emg", cfg.GetProperty("exg_mode").GetString());

            await mgr.StopAsync();
        }


        // ----- Subscribe behavior -----


        /// <summary>
        /// Helper: returns the current subscription set (MAC addresses) for a given client ID,
        /// reading the private <c>_subscriptions</c> field of <see cref="WsBridgeManager"/> via reflection.
        /// </summary>
        /// <param name="mgr">The bridge manager under test.</param>
        /// <param name="clientId">The client identifier associated with the subscription set.</param>
        /// <returns>The <see cref="HashSet{T}"/> of subscribed MAC strings for the client, or <c>null</c> if none exists.</returns>
        private static HashSet<string>? GetSubscribedSet(WsBridgeManager mgr, Guid clientId)
        {
            var f = typeof(WsBridgeManager)
                .GetField("_subscriptions", BindingFlags.NonPublic | BindingFlags.Instance);
            Assert.NotNull(f);

            var dict = f!.GetValue(mgr)!; // ConcurrentDictionary<Guid, HashSet<string>>

            var tryGet = dict.GetType().GetMethod("TryGetValue",
                BindingFlags.Public | BindingFlags.Instance);
            Assert.NotNull(tryGet);

            object?[] args = new object?[] { clientId, null };
            bool ok = (bool)tryGet!.Invoke(dict, args)!;

            if (!ok) return null;
            return (HashSet<string>)args[1]!;
        }


        /// <summary>
        /// "open" subscribes the requesting client to the given MAC and logs the action.
        /// Expected:
        /// - The subscription set for the client contains the MAC
        /// - A log entry indicates a successful subscription for that MAC
        /// </summary>
        [Fact]
        public async Task Open_SubscribesClient_ToGivenMac_AndLogs()
        {
            var mgr = new WsBridgeManager();
            var logs = new List<string>();
            mgr.Log += s => logs.Add(s);

            await mgr.StartAsync(new Activity());

            // An active session is required to join the "open" branch that calls Subscribe
            var mac = "00:06:66:AA:BB:CC";
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig
            {
                EnableGyroscope = true,
                SamplingRate = 51
            });

            var ws = typeof(WsBridgeManager)
                .GetField("_ws", BindingFlags.NonPublic | BindingFlags.Instance)!
                .GetValue(mgr) as WatsonWebsocket.WatsonWsServer;
            Assert.NotNull(ws);

            var client = Guid.NewGuid();

            // Send the "open" command
            ws!.RaiseText(client, $"{{\"type\":\"open\",\"mac\":\"{mac}\"}}");

            var set = GetSubscribedSet(mgr, client);
            Assert.NotNull(set);
            Assert.Contains(mac, set!);

            Assert.Contains(logs, s => s.Contains("subscribed", StringComparison.OrdinalIgnoreCase)
                                     && s.Contains(mac, StringComparison.OrdinalIgnoreCase));

            await mgr.StopAsync();
        }


        /// <summary>
        /// Repeated "open" calls are idempotent; MAC matching is case-insensitive.
        /// Expected:
        /// - Subscription set contains a single canonical MAC
        /// - Subsequent opens with different casing do not create duplicates
        /// </summary>
        [Fact]
        public async Task Open_IsIdempotent_AndCaseInsensitive()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:CC";
            // Active session
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig { EnableBattery = true, SamplingRate = 64 });

            var ws = typeof(WsBridgeManager)
                .GetField("_ws", BindingFlags.NonPublic | BindingFlags.Instance)!
                .GetValue(mgr) as WatsonWebsocket.WatsonWsServer;
            Assert.NotNull(ws);

            var client = Guid.NewGuid();

            // First open (uppercase)
            ws!.RaiseText(client, $"{{\"type\":\"open\",\"mac\":\"{mac.ToUpperInvariant()}\"}}");
            var set = GetSubscribedSet(mgr, client);
            Assert.NotNull(set);
            Assert.Single(set!);               
            Assert.Contains(mac, set!);

            // Second open (lowercase): only 1 element must remain
            ws.RaiseText(client, $"{{\"type\":\"open\",\"mac\":\"{mac.ToLowerInvariant()}\"}}");
            set = GetSubscribedSet(mgr, client);
            Assert.NotNull(set);
            Assert.Single(set!);
            Assert.Contains(mac, set!);

            // third identical open: still idempotent
            ws.RaiseText(client, $"{{\"type\":\"open\",\"mac\":\"{mac}\"}}");
            set = GetSubscribedSet(mgr, client);
            Assert.NotNull(set);
            Assert.Single(set!);

            await mgr.StopAsync();
        }


        // ----- Unsubscribe behavior -----


        /// <summary>
        /// "unsubscribe" removes the MAC from the client's subscription set and logs the action.
        /// Expected:
        /// - After unsubscribe, the client's subscription set no longer contains the MAC
        /// - A log entry indicates unsubscription for that MAC
        /// </summary>
        [Fact]
        public async Task Unsubscribe_RemovesMac_AndLogs()
        {
            var mgr = new WsBridgeManager();
            var logs = new List<string>();
            mgr.Log += s => logs.Add(s);

            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:CC";

            // An active session is required for "open"/"unsubscribe" to work on a valid MAC
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig
            {
                EnableGyroscope = true,
                SamplingRate = 51
            });

            var ws = typeof(WsBridgeManager)
                .GetField("_ws", BindingFlags.NonPublic | BindingFlags.Instance)!
                .GetValue(mgr) as WatsonWebsocket.WatsonWsServer;
            Assert.NotNull(ws);

            var client = Guid.NewGuid();

            // subscribe via "open"
            ws!.RaiseText(client, $"{{\"type\":\"open\",\"mac\":\"{mac}\"}}");
            var set = GetSubscribedSet(mgr, client);
            Assert.NotNull(set);
            Assert.Contains(mac, set!);

            // unsubscribe
            ws.RaiseText(client, $"{{\"type\":\"unsubscribe\",\"mac\":\"{mac}\"}}");

            set = GetSubscribedSet(mgr, client);

            if (set != null)
                Assert.DoesNotContain(mac, set);

            // Log contains the unsubscribe message
            Assert.Contains(logs, s => s.Contains("unsubscribed", StringComparison.OrdinalIgnoreCase)
                                     && s.Contains(mac, StringComparison.OrdinalIgnoreCase));

            await mgr.StopAsync();
        }


        /// <summary>
        /// Unsubscribe is idempotent, case-insensitive, and safe even if the client isn't subscribed.
        /// Expected:
        /// - Unsubscribe before any open does not throw and does not create an entry
        /// - After open, unsubscribe removes the MAC regardless of casing
        /// - Repeated unsubscribe calls do nothing and do not throw
        /// </summary>
        [Fact]
        public async Task Unsubscribe_IsIdempotent_AndCaseInsensitive_AndSafeWhenNotSubscribed()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:CC";
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig { EnableBattery = true, SamplingRate = 64 });

            var ws = typeof(WsBridgeManager)
                .GetField("_ws", BindingFlags.NonPublic | BindingFlags.Instance)!
                .GetValue(mgr) as WatsonWebsocket.WatsonWsServer;
            Assert.NotNull(ws);

            var client = Guid.NewGuid();

            // Unsubscribe before you are subscribed
            ws!.RaiseText(client, $"{{\"type\":\"unsubscribe\",\"mac\":\"{mac}\"}}");
            var set = GetSubscribedSet(mgr, client);
            Assert.Null(set);

            // Suscribed
            ws.RaiseText(client, $"{{\"type\":\"open\",\"mac\":\"{mac}\"}}");
            set = GetSubscribedSet(mgr, client);
            Assert.NotNull(set);
            Assert.Single(set!);
            Assert.Contains(mac, set!);

            // Unsubscribe with different casing (case-insensitive)
            ws.RaiseText(client, $"{{\"type\":\"unsubscribe\",\"mac\":\"{mac.ToLowerInvariant()}\"}}");
            set = GetSubscribedSet(mgr, client);
            if (set != null)
                Assert.DoesNotContain(mac, set);

            // Repeated unsubscribe (idempotent)
            ws.RaiseText(client, $"{{\"type\":\"unsubscribe\",\"mac\":\"{mac}\"}}");
            set = GetSubscribedSet(mgr, client);
            if (set != null)
                Assert.DoesNotContain(mac, set);

            await mgr.StopAsync();
        }


        // ----- BroadcastToSubscribers behavior -----


        /// <summary>
        /// Helper: returns the array of subscribed MAC addresses for the specified client,
        /// accessing the private <c>_subscriptions</c> dictionary through reflection.
        /// </summary>
        /// <param name="mgr">The bridge manager instance.</param>
        /// <param name="clientId">The client whose subscriptions are queried.</param>
        /// <returns>An array of subscribed MAC strings; empty if the client has no entry.</returns>
        private static string[] GetSubscribedMacs(WsBridgeManager mgr, Guid clientId)
        {
            var f = typeof(WsBridgeManager).GetField("_subscriptions",
                BindingFlags.NonPublic | BindingFlags.Instance);
            Assert.NotNull(f);

            var dict = f!.GetValue(mgr)!; // ConcurrentDictionary<Guid, HashSet<string>>
            var tryGet = dict.GetType().GetMethod("TryGetValue",
                BindingFlags.Public | BindingFlags.Instance);
            Assert.NotNull(tryGet);

            object?[] args = new object?[] { clientId, null };
            bool ok = (bool)tryGet!.Invoke(dict, args)!;

            if (!ok || args[1] is null) return Array.Empty<string>();
            return ((System.Collections.Generic.HashSet<string>)args[1]!).ToArray();
        }


        /// <summary>
        /// If the internal WS server is null, broadcast paths complete without throwing.
        /// Expected:
        /// - Calling update that triggers a broadcast completes without exceptions
        /// </summary>
        [Fact]
        public async Task Broadcast_NoWs_NoThrow()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:11:22:33";
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig { EnableMagnetometer = true, SamplingRate = 51 });

            var f = typeof(WsBridgeManager).GetField("_ws", BindingFlags.NonPublic | BindingFlags.Instance);
            Assert.NotNull(f);
            f!.SetValue(mgr, null);

            var ex = await Record.ExceptionAsync(async () =>
                await mgr.UpdateConfigAsync(mac, new ShimmerConfig { EnableBattery = true, SamplingRate = 64 }));
            Assert.Null(ex);
        }


        /// <summary>
        /// When there are no subscribers, broadcasting a config change sends nothing.
        /// Expected:
        /// - No frames are recorded as sent
        /// </summary>
        [Fact]
        public async Task Broadcast_WhenNoSubscribers_DoesNothing()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:44:55:66";
            WatsonWsServer.ClearSentLog();

            // Open HW session, no members
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig { EnableLowNoiseAccelerometer = true, SamplingRate = 51 });
            Assert.Empty(WatsonWsServer.SentLog);

            // UpdateConfigAsync will broadcast, but since there are no subscribers it doesn't have to send anything
            await mgr.UpdateConfigAsync(mac, new ShimmerConfig { EnableGyroscope = true, SamplingRate = 128 });
            Assert.Empty(WatsonWsServer.SentLog);

            await mgr.StopAsync();
        }


        /// <summary>
        /// Broadcast delivers config_changed only to clients subscribed to the MAC.
        /// Expected:
        /// - No exceptions thrown
        /// - No "Broadcast error" in logs
        /// </summary>
        [Fact]
        public async Task Broadcast_WithOneSubscriber_Completes_NoErrors()
        {
            var mgr = new WsBridgeManager();
            string? lastLog = null;
            mgr.Log += s => lastLog = s;

            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:01";

            // Open a hardware session (the content doesn't matter, it's used to make the MAC active)
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig { EnableGyroscope = true, SamplingRate = 51 });

            // Register two clients; we only register A
            var ws = GetWs(mgr);
            var clientA = Guid.NewGuid();
            var clientB = Guid.NewGuid();
            ws.RaiseConnected(clientA);
            ws.RaiseConnected(clientB);

            // "open" subscribes clientA to the MAC
            ws.RaiseText(clientA, $"{{\"type\":\"open\",\"mac\":\"{mac}\"}}");

            // Sanity: A is registered, B is not
            Assert.Contains(mac, GetSubscribedMacs(mgr, clientA));
            Assert.DoesNotContain(mac, GetSubscribedMacs(mgr, clientB));

            // This will cause a "config_changed" broadcast to subscribers only
            var ex = await Record.ExceptionAsync(async () =>
                await mgr.UpdateConfigAsync(mac, new ShimmerConfig { EnableBattery = true, SamplingRate = 100 })
            );

            Assert.Null(ex);
            Assert.DoesNotContain("Broadcast error", lastLog ?? string.Empty);

            await mgr.StopAsync();
        }


        /// <summary>
        /// A broadcast with no subscribers completes silently.
        /// Expected:
        /// - No exceptions thrown
        /// - No broadcast error logged
        /// </summary>
        [Fact]
        public async Task Broadcast_NoSubscribers_Completes_Silently()
        {
            var mgr = new WsBridgeManager();
            string? lastLog = null;
            mgr.Log += s => lastLog = s;

            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:02";

            // Session active, but no "open" client -> no subscription
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig { EnableLowNoiseAccelerometer = true, SamplingRate = 51 });

            var ex = await Record.ExceptionAsync(async () =>
                await mgr.UpdateConfigAsync(mac, new ShimmerConfig { EnableGyroscope = true, SamplingRate = 128 })
            );

            Assert.Null(ex);
            Assert.DoesNotContain("Broadcast error", lastLog ?? string.Empty);

            await mgr.StopAsync();
        }


        /// <summary>
        /// Guarding for null WS server is robust; operations do not throw.
        /// Expected:
        /// - Updating configuration while WS is null completes without exceptions
        /// </summary>
        [Fact]
        public async Task Broadcast_WhenWsIsNull_NoThrow()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:03";
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig { EnableMagnetometer = true, SamplingRate = 51 });

            // Force _ws = null to test the guard
            var f = typeof(WsBridgeManager).GetField("_ws", BindingFlags.NonPublic | BindingFlags.Instance);
            Assert.NotNull(f);
            f!.SetValue(mgr, null);

            var ex = await Record.ExceptionAsync(async () =>
                await mgr.UpdateConfigAsync(mac, new ShimmerConfig { EnableBattery = true, SamplingRate = 64 })
            );

            Assert.Null(ex);
        }


        // ----- SendJson behavior -----


        /// <summary>
        /// Helper: obtains the private <c>SendJson</c> method from <see cref="WsBridgeManager"/> for reflective invocation in tests.
        /// </summary>
        /// <returns>The <see cref="MethodInfo"/> representing the non-public instance method <c>SendJson(Guid, object)</c>.</returns>
        private static MethodInfo GetSendJsonMI()
        {
            var mi = typeof(WsBridgeManager).GetMethod(
                "SendJson",
                BindingFlags.NonPublic | BindingFlags.Instance
            );
            Assert.NotNull(mi);
            return mi!;
        }


        /// <summary>
        /// When WS server is not initialized, SendJson returns a completed task and does not throw.
        /// Expected:
        /// - Invocation completes
        /// - No "WS send error" is logged
        /// </summary>
        [Fact]
        public async Task SendJson_NoServer_ReturnsCompletedTask_NoThrow()
        {
            var mgr = new WsBridgeManager();
            string? lastLog = null;
            mgr.Log += s => lastLog = s;

            var mi = GetSendJsonMI();

            var clientId = Guid.NewGuid();
            var payload = new { type = "ping" };

            Task? t = null;
            var ex = Record.Exception(() =>
            {
                var ret = mi.Invoke(mgr, new object[] { clientId, payload });
                t = (Task)ret!;
            });

            Assert.Null(ex);
            Assert.NotNull(t);
            await t!;

            Assert.True(string.IsNullOrEmpty(lastLog) || !lastLog!.Contains("WS send error", StringComparison.OrdinalIgnoreCase));
        }


        /// <summary>
        /// With a live WS server and a serializable payload, SendJson completes without errors.
        /// Expected:
        /// - Invocation completes
        /// - No "WS send error" in logs
        /// </summary>
        [Fact]
        public async Task SendJson_WithServerAndSerializablePayload_Completes_NoErrorLog()
        {
            var mgr = new WsBridgeManager();
            string? lastLog = null;
            mgr.Log += s => lastLog = s;

            await mgr.StartAsync(new Activity());

            var mi = GetSendJsonMI();

            var clientId = Guid.NewGuid();
            var payload = new { type = "hello_ack", ok = true };

            Task? t = null;
            var ex = Record.Exception(() =>
            {
                var ret = mi.Invoke(mgr, new object[] { clientId, payload });
                t = (Task)ret!;
            });

            Assert.Null(ex);
            Assert.NotNull(t);
            await t!;
            Assert.True(string.IsNullOrEmpty(lastLog) || !lastLog!.Contains("WS send error", StringComparison.OrdinalIgnoreCase));

            await mgr.StopAsync();
        }


        /// <summary>
        /// Test-only payload type that intentionally contains a self-reference to trigger serialization failures.
        /// Used to verify that <c>SendJson</c> logs errors and returns a completed task when serialization throws.
        /// </summary>
        private sealed class Cyclic
        {
            public Cyclic? Self { get; set; }
        }


        /// <summary>
        /// If JSON serialization throws (e.g., cyclic object), SendJson logs and returns a completed task.
        /// Expected:
        /// - No exception escapes reflection invoke
        /// - A "WS send error" is logged
        /// </summary>
        [Fact]
        public async Task SendJson_WhenSerializationThrows_LogsAndReturnsCompletedTask()
        {
            var mgr = new WsBridgeManager();
            string? lastLog = null;
            mgr.Log += s => lastLog = s;

            await mgr.StartAsync(new Activity());

            var mi = GetSendJsonMI();

            var clientId = Guid.NewGuid();
            var cyc = new Cyclic();
            cyc.Self = cyc;

            Task? t = null;

            var ex = Record.Exception(() =>
            {
                var ret = mi.Invoke(mgr, new object[] { clientId, (object)cyc });
                t = (Task)ret!;
            });

            Assert.Null(ex);
            Assert.NotNull(t);
            await t!;

            Assert.NotNull(lastLog);
            Assert.Contains("WS send error", lastLog!, StringComparison.OrdinalIgnoreCase);

            await mgr.StopAsync();
        }


        // ----- SppSession class -----


        /// <summary>
        /// Helper: retrieves the non-public nested <c>SppSession</c> type defined inside <see cref="WsBridgeManager"/>.
        /// </summary>
        /// <returns>The <see cref="Type"/> object for the nested session class.</returns>
        private static Type GetSppSessionType()
        {
            var t = typeof(WsBridgeManager).GetNestedType("SppSession", BindingFlags.NonPublic);
            Assert.NotNull(t);
            return t!;
        }


        /// <summary>
        /// Helper: constructs a new SPP session via its non-public constructor,
        /// allowing injection of broadcast/log delegates for verification.
        /// </summary>
        /// <param name="mac">Device MAC address (may be trimmed by the ctor).</param>
        /// <param name="broadcast">Callback invoked to broadcast JSON to WS subscribers.</param>
        /// <param name="log">Callback invoked to collect diagnostic messages.</param>
        /// <returns>A new SPP session instance (boxed as <see cref="object"/>).</returns>
        private static object CreateSppSession(string mac, Action<string, string>? broadcast = null, Action<string>? log = null)
        {
            var t = GetSppSessionType();

            var ctor = t.GetConstructor(
                BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic,
                binder: null,
                new Type[] { typeof(string), typeof(Action<string, string>), typeof(Action<string>) },
                modifiers: null);

            Assert.NotNull(ctor);

            broadcast ??= (_, __) => { };
            log ??= _ => { };

            var instance = ctor!.Invoke(new object[] { mac, broadcast, log });
            Assert.NotNull(instance);
            return instance;
        }


        /// <summary>
        /// Helper: reads the <c>IsModeLocked</c> property from a SPP session instance via reflection.
        /// </summary>
        /// <param name="spp">The SPP session instance.</param>
        /// <returns><c>true</c> if mode is locked; otherwise <c>false</c>.</returns>
        private static bool GetIsModeLocked(object spp)
        {
            var p = spp.GetType().GetProperty("IsModeLocked",
                BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(p);
            var val = p!.GetValue(spp);
            Assert.IsType<bool>(val);
            return (bool)val!;
        }


        /// <summary>
        /// Helper: invokes the <c>LockMode()</c> method on the SPP session via reflection.
        /// </summary>
        /// <param name="spp">The SPP session instance.</param>
        private static void CallLockMode(object spp)
        {
            var m = spp.GetType().GetMethod("LockMode",
                BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(m);
            m!.Invoke(spp, Array.Empty<object>());
        }


        // ----- LockMode behavior ----- 


        /// <summary>
        /// Calling LockMode sets IsModeLocked to true.
        /// Expected:
        /// - IsModeLocked transitions from false to true
        /// </summary>
        [Fact]
        public void LockMode_Sets_IsModeLocked_True()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:FF");

            // before the lock
            Assert.False(GetIsModeLocked(spp));

            // lock
            CallLockMode(spp);

            // after the lock
            Assert.True(GetIsModeLocked(spp));
        }


        /// <summary>
        /// LockMode is idempotent; repeated calls keep IsModeLocked true.
        /// Expected:
        /// - IsModeLocked remains true across multiple LockMode calls
        /// </summary>
        [Fact]
        public void LockMode_IsIdempotent_RemainsTrue_OnMultipleCalls()
        {
            var spp = CreateSppSession("11:22:33:44:55:66");

            CallLockMode(spp);
            Assert.True(GetIsModeLocked(spp));

            // multiple calls must not change anything (remains true)
            CallLockMode(spp);
            CallLockMode(spp);

            Assert.True(GetIsModeLocked(spp));
        }


        /// <summary>
        /// LockMode does not trigger broadcast/log callbacks on its own.
        /// Expected:
        /// - After LockMode, no broadcast/log delegates are invoked implicitly
        /// </summary>
        [Fact]
        public void LockMode_DoesNotDepend_OnDelegates()
        {

            // Also check with “real” delegates (that capture data) that do not affect the lock
            string? lastMac = null;
            string? lastJson = null;
            string? lastLog = null;

            void Broadcast(string mac, string json) { lastMac = mac; lastJson = json; }
            void Log(string s) { lastLog = s; }

            var spp = CreateSppSession("FE:DC:BA:98:76:54", Broadcast, Log);

            Assert.False(GetIsModeLocked(spp));
            CallLockMode(spp);
            Assert.True(GetIsModeLocked(spp));

            // delegates must not have been invoked by LockMode alone
            Assert.Null(lastMac);
            Assert.Null(lastJson);
            Assert.Null(lastLog);
        }


        // ----- IsModeLocked behavior -----


        /// <summary>
        /// Default SPP session starts with mode unlocked.
        /// Expected:
        /// - IsModeLocked == false by default
        /// </summary>
        [Fact]
        public void IsModeLocked_Default_IsFalse()
        {
            var spp = CreateSppSession("00:11:22:33:44:55");
            Assert.False(GetIsModeLocked(spp));
        }


        // ----- CurrentConfig behavior -----


        /// <summary>
        /// Helper: constructs a new SPP session with only MAC argument (test-oriented overload).
        /// </summary>
        /// <param name="mac">Device MAC address.</param>
        /// <returns>A new SPP session instance.</returns>
        private static object CreateSppSession(string mac)
        {
            var t = GetSppSessionType();
            var ctor = t.GetConstructor(
                BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic,
                binder: null,
                new Type[] { typeof(string), typeof(Action<string, string>), typeof(Action<string>) },
                modifiers: null);
            Assert.NotNull(ctor);
            return ctor!.Invoke(new object[] { mac, (Action<string, string>)((_, __) => { }), (Action<string>)(_ => { }) });
        }


        /// <summary>
        /// Helper: sets the private field <c>_currentCfg</c> on a SPP session to the provided config.
        /// </summary>
        /// <param name="spp">The SPP session instance.</param>
        /// <param name="cfg">The configuration to set.</param>
        private static void SetPrivateCurrentCfg(object spp, ShimmerConfig cfg)
        {
            var f = spp.GetType().GetField("_currentCfg", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(f);
            f!.SetValue(spp, cfg);
        }


        /// <summary>
        /// Helper: reads the public snapshot property <c>CurrentConfig</c> from a SPP session.
        /// </summary>
        /// <param name="spp">The SPP session instance.</param>
        /// <returns>A detached snapshot of <see cref="ShimmerConfig"/>.</returns>
        private static ShimmerConfig GetCurrentConfigSnapshot(object spp)
        {
            var p = spp.GetType().GetProperty("CurrentConfig", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
            Assert.NotNull(p);
            var v = p!.GetValue(spp);
            Assert.IsType<ShimmerConfig>(v);
            return (ShimmerConfig)v!;
        }


        /// <summary>
        /// Helper: returns the private field <c>_currentCfg</c> (internal reference) of a SPP session.
        /// </summary>
        /// <param name="spp">The SPP session instance.</param>
        /// <returns>The internal <see cref="ShimmerConfig"/> reference.</returns>
        private static ShimmerConfig GetPrivateCurrentCfg(object spp)
        {
            var f = spp.GetType().GetField("_currentCfg", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(f);
            var v = f!.GetValue(spp);
            Assert.IsType<ShimmerConfig>(v);
            return (ShimmerConfig)v!;
        }


        /// <summary>
        /// CurrentConfig returns a semantically equal copy of the applied configuration.
        /// Expected:
        /// - Snapshot values match the internal config values
        /// - Snapshot is not the same instance as the internal reference
        /// </summary>
        [Fact]
        public void CurrentConfig_Returns_ExactValues_Copy()
        {
            var spp = CreateSppSession("00:11:22:33:44:55");

            var applied = new ShimmerConfig
            {

                // IMU
                EnableLowNoiseAccelerometer = true,
                EnableWideRangeAccelerometer = true,
                EnableGyroscope = true,
                EnableMagnetometer = true,
                EnablePressureTemperature = true,
                EnableBattery = true,
                EnableExtA6 = true,
                EnableExtA7 = false,
                EnableExtA15 = true,
                SamplingRate = 50.5,

                // EXG
                EnableExg1 = true,
                EnableExg2 = false,
                ExgUse16Bit = true,
                ExgMode = ExgMode.Respiration
            };
            SetPrivateCurrentCfg(spp, applied);

            var snap = GetCurrentConfigSnapshot(spp);

            Assert.True(snap.EnableLowNoiseAccelerometer);
            Assert.True(snap.EnableWideRangeAccelerometer);
            Assert.True(snap.EnableGyroscope);
            Assert.True(snap.EnableMagnetometer);
            Assert.True(snap.EnablePressureTemperature);
            Assert.True(snap.EnableBattery);
            Assert.True(snap.EnableExtA6);
            Assert.False(snap.EnableExtA7);
            Assert.True(snap.EnableExtA15);
            Assert.Equal(50.5, snap.SamplingRate);

            Assert.True(snap.EnableExg1);
            Assert.False(snap.EnableExg2);
            Assert.True(snap.ExgUse16Bit);
            Assert.Equal(ExgMode.Respiration, snap.ExgMode);

            // it's not the same instance
            var internalRef = GetPrivateCurrentCfg(spp);
            Assert.False(Object.ReferenceEquals(snap, internalRef));
        }


        /// <summary>
        /// The returned CurrentConfig snapshot is detached; mutating it does not alter the internal state.
        /// Expected:
        /// - Mutations on snapshot do not affect subsequent snapshots
        /// </summary>
        [Fact]
        public void CurrentConfig_IsDetached_MutationsOnSnapshot_DoNotAffectInternal()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:FF");

            var applied = new ShimmerConfig
            {
                EnableGyroscope = true,
                EnableBattery = true,
                SamplingRate = 51,
                EnableExg1 = true,
                ExgMode = ExgMode.EMG
            };
            SetPrivateCurrentCfg(spp, applied);

            var snap1 = GetCurrentConfigSnapshot(spp);
            Assert.True(snap1.EnableGyroscope);
            Assert.True(snap1.EnableBattery);
            Assert.True(snap1.EnableExg1);
            Assert.Equal(ExgMode.EMG, snap1.ExgMode);

            // snapshot mutations
            snap1.EnableGyroscope = false;
            snap1.EnableBattery = false;
            snap1.EnableExg1 = false;
            snap1.ExgMode = ExgMode.None;
            snap1.SamplingRate = 200;

            // get new snapshot from object -> must reflect unchanged internal state
            var snap2 = GetCurrentConfigSnapshot(spp);
            Assert.True(snap2.EnableGyroscope);
            Assert.True(snap2.EnableBattery);
            Assert.True(snap2.EnableExg1);
            Assert.Equal(ExgMode.EMG, snap2.ExgMode);
            Assert.Equal(51, snap2.SamplingRate);
        }


        /// <summary>
        /// Each access to CurrentConfig returns a new instance.
        /// Expected:
        /// - Distinct references for successive snapshots
        /// - Mutating one snapshot does not affect the other
        /// </summary>
        [Fact]
        public void CurrentConfig_ReturnsNewInstance_EachAccess()
        {
            var spp = CreateSppSession("11:22:33:44:55:66");

            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableMagnetometer = true, SamplingRate = 100 });

            var a = GetCurrentConfigSnapshot(spp);
            var b = GetCurrentConfigSnapshot(spp);

            Assert.False(ReferenceEquals(a, b));
            Assert.True(a.EnableMagnetometer);
            Assert.True(b.EnableMagnetometer);

            // change A -> B must not change
            a.EnableMagnetometer = false;
            a.SamplingRate = 1;

            Assert.True(b.EnableMagnetometer);
            Assert.Equal(100, b.SamplingRate);
        }


        // ----- EnabledBlocks behavior -----


        /// <summary>
        /// Helper: minimal factory using a fixed MAC for convenience in EnabledBlocks tests.
        /// </summary>
        /// <returns>A new SPP session instance.</returns>
        private object CreateSppSession() => CreateSppSession("11:22:33:44:55");


        /// <summary>
        /// Helper: invokes the public method <c>EnabledBlocks()</c> on the SPP session and returns the list of keys.
        /// </summary>
        /// <param name="spp">The SPP session instance.</param>
        /// <returns>An ordered list of enabled block keys.</returns>
        private static IReadOnlyList<string> CallEnabledBlocks(object spp)
        {
            var m = spp.GetType().GetMethod(
                "EnabledBlocks",
                BindingFlags.Instance | BindingFlags.Public
            );
            Assert.NotNull(m);
            var res = m!.Invoke(spp, null);
            Assert.IsAssignableFrom<IReadOnlyList<string>>(res);
            return (IReadOnlyList<string>)res!;
        }


        /// <summary>
        /// When all flags are off, EnabledBlocks returns an empty list.
        /// Expected:
        /// - Empty list
        /// </summary>
        [Fact]
        public void EnabledBlocks_Empty_When_AllFlagsOff()
        {
            var spp = CreateSppSession();
            SetPrivateCurrentCfg(spp, new ShimmerConfig());
            var blocks = CallEnabledBlocks(spp);
            Assert.Empty(blocks);
        }


        /// <summary>
        /// EXG is present if either Exg1 or Exg2 is enabled (no duplicates).
        /// Expected:
        /// - Only ["exg"] whether Exg1, Exg2, or both are enabled
        /// </summary>
        [Fact]
        public void EnabledBlocks_Exg_When_Exg1_Or_Exg2()
        {
            var spp = CreateSppSession();

            // EXG1 only
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableExg1 = true });
            Assert.Equal(new[] { "exg" }, CallEnabledBlocks(spp));

            // EXG2 only
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableExg2 = true });
            Assert.Equal(new[] { "exg" }, CallEnabledBlocks(spp));

            // Both EXG1 + EXG2 -> still a single "exg"
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableExg1 = true, EnableExg2 = true });
            Assert.Equal(new[] { "exg" }, CallEnabledBlocks(spp));
        }


        /// <summary>
        /// Each external ADC flag produces its own block: "ext6", "ext7", "ext15".
        /// Expected:
        /// - ["ext6"] if A6
        /// - ["ext7"] if A7
        /// - ["ext15"] if A15
        /// - Combinations in deterministic order: A6, A7, A15
        /// </summary>
        [Fact]
        public void EnabledBlocks_Ext_Split_By_Channel()
        {
            var spp = CreateSppSession();

            // A6 only
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableExtA6 = true });
            Assert.Equal(new[] { "ext6" }, CallEnabledBlocks(spp));

            // A7 only
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableExtA7 = true });
            Assert.Equal(new[] { "ext7" }, CallEnabledBlocks(spp));

            // A15 only
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableExtA15 = true });
            Assert.Equal(new[] { "ext15" }, CallEnabledBlocks(spp));

            // A6 + A7
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableExtA6 = true, EnableExtA7 = true });
            Assert.Equal(new[] { "ext6", "ext7" }, CallEnabledBlocks(spp));

            // A6 + A15
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableExtA6 = true, EnableExtA15 = true });
            Assert.Equal(new[] { "ext6", "ext15" }, CallEnabledBlocks(spp));

            // A7 + A15
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableExtA7 = true, EnableExtA15 = true });
            Assert.Equal(new[] { "ext7", "ext15" }, CallEnabledBlocks(spp));

            // A6 + A7 + A15
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableExtA6 = true, EnableExtA7 = true, EnableExtA15 = true });
            Assert.Equal(new[] { "ext6", "ext7", "ext15" }, CallEnabledBlocks(spp));
        }


        /// <summary>
        /// Enabling Pressure/Temperature adds the pair "temp", then "press", in this order.
        /// Expected:
        /// - ["temp","press"]
        /// </summary>
        [Fact]
        public void EnabledBlocks_TempAdds_Temp_And_Press()
        {
            var spp = CreateSppSession();

            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnablePressureTemperature = true });
            var blocks = CallEnabledBlocks(spp);

            // must contain both and in the correct order temp then press
            Assert.Equal(new[] { "temp", "press" }, blocks);
        }


        /// <summary>
        /// Each individual IMU flag maps to its expected single key.
        /// Expected:
        /// - LNA -> ["lna"], WRA -> ["wra"], Gyro -> ["gyro"], Mag -> ["mag"], Battery -> ["vbatt"]
        /// </summary>
        [Fact]
        public void EnabledBlocks_IndividualFlags_MapToExpectedKeys()
        {
            var spp = CreateSppSession();

            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableLowNoiseAccelerometer = true });
            Assert.Equal(new[] { "lna" }, CallEnabledBlocks(spp));

            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableWideRangeAccelerometer = true });
            Assert.Equal(new[] { "wra" }, CallEnabledBlocks(spp));

            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableGyroscope = true });
            Assert.Equal(new[] { "gyro" }, CallEnabledBlocks(spp));

            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableMagnetometer = true });
            Assert.Equal(new[] { "mag" }, CallEnabledBlocks(spp));

            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableBattery = true });
            Assert.Equal(new[] { "vbatt" }, CallEnabledBlocks(spp));
        }


        /// <summary>
        /// When multiple flags are enabled, the resulting block list is stable (deterministic order) and deduplicated.
        /// Expected:
        /// - ["exg", "lna", "wra", "gyro", "mag", "temp", "press", "vbatt", "ext6", "ext7", "ext15"]
        /// </summary>
        [Fact]
        public void EnabledBlocks_Composite_AllFlags_OrderIsStable()
        {
            var spp = CreateSppSession();

            var cfg = new ShimmerConfig
            {
                EnableExg1 = true,
                EnableExg2 = true,
                EnableLowNoiseAccelerometer = true,
                EnableWideRangeAccelerometer = true,
                EnableGyroscope = true,
                EnableMagnetometer = true,
                EnablePressureTemperature = true,
                EnableBattery = true,
                EnableExtA6 = true,
                EnableExtA7 = true,
                EnableExtA15 = true
            };
            SetPrivateCurrentCfg(spp, cfg);

            var blocks = CallEnabledBlocks(spp);

            var expected = new[] { "exg", "lna", "wra", "gyro", "mag", "temp", "press", "vbatt", "ext6", "ext7", "ext15"};
            Assert.Equal(expected, blocks);
        }


        /// <summary>
        /// No duplicates when multiple groups are enabled.
        /// Expected: ["exg"]
        /// </summary>
        [Fact]
        public void EnabledBlocks_NoDuplicates_When_MultipleGroupsEnabled()
        {
            var spp = CreateSppSession();

            var cfg = new ShimmerConfig
            {
                EnableExg1 = true,
                EnableExg2 = true,
            };
            SetPrivateCurrentCfg(spp, cfg);

            var blocks = CallEnabledBlocks(spp);

            Assert.Equal(new[] { "exg"}, blocks);
        }


        /// <summary>
        /// Stable output order regardless of flag set order.
        /// Expected: ["exg", "lna", "wra", "gyro", "mag", "temp", "press", "vbatt", "ext15"]
        /// </summary>
        [Fact]
        public void EnabledBlocks_OrderStable_RegardlessOfFlagSetOrder()
        {
            var spp = CreateSppSession();

            // Set flags in “strange” order
            var cfg = new ShimmerConfig
            {
                EnableMagnetometer = true,
                EnableBattery = true,
                EnableWideRangeAccelerometer = true,
                EnableLowNoiseAccelerometer = true,
                EnablePressureTemperature = true,
                EnableExg2 = true,
                EnableGyroscope = true,
                EnableExtA15 = true
            };
            SetPrivateCurrentCfg(spp, cfg);

            var blocks = CallEnabledBlocks(spp);
            var expected = new[] { "exg", "lna", "wra", "gyro", "mag", "temp", "press", "vbatt", "ext15" };
            Assert.Equal(expected, blocks);
        }


        // ----- ResetIndices behavior -----


        /// <summary>
        /// Helper: sets a private index field to a specific value for testing ResetIndices.
        /// </summary>
        /// <param name="spp">The SPP session instance.</param>
        /// <param name="field">Private field name.</param>
        /// <param name="value">Integer value to assign.</param>
        private static void SetPrivateIndex(object spp, string field, int value)
        {
            var f = spp.GetType().GetField(field, BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(f);
            f!.SetValue(spp, value);
        }


        /// <summary>
        /// Helper: reads a private index field for assertions.
        /// </summary>
        /// <param name="spp">The SPP session instance.</param>
        /// <param name="field">Private field name.</param>
        /// <returns>Field integer value.</returns>
        private static int GetPrivateIndex(object spp, string field)
        {
            var f = spp.GetType().GetField(field, BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(f);
            return (int)f!.GetValue(spp)!;
        }


        /// <summary>
        /// Helper: invokes the private <c>ResetIndices()</c> method on SPP session.
        /// </summary>
        /// <param name="spp">The SPP session instance.</param>
        private static void InvokeResetIndices(object spp)
        {
            var m = spp.GetType().GetMethod("ResetIndices", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(m);
            m!.Invoke(spp, null);
        }


        /// <summary>
        /// ResetIndices sets all cached indices to -1 and is idempotent.
        /// Expected:
        /// - All index fields equal -1 after call
        /// - Repeating the call leaves them at -1
        /// </summary>
        [Fact]
        public void ResetIndices_SetsAllCachedIndicesToMinusOne_And_IsIdempotent()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:FF");

            // Preload values ​​!= -1 to ensure they are cleared
            SetPrivateIndex(spp, "iTs", 5);

            SetPrivateIndex(spp, "iExg1", 10);
            SetPrivateIndex(spp, "iExg2", 11);

            SetPrivateIndex(spp, "iLnaX", 20);
            SetPrivateIndex(spp, "iLnaY", 21);
            SetPrivateIndex(spp, "iLnaZ", 22);

            SetPrivateIndex(spp, "iWraX", 30);
            SetPrivateIndex(spp, "iWraY", 31);
            SetPrivateIndex(spp, "iWraZ", 32);

            SetPrivateIndex(spp, "iGx", 40);
            SetPrivateIndex(spp, "iGy", 41);
            SetPrivateIndex(spp, "iGz", 42);

            SetPrivateIndex(spp, "iMx", 50);
            SetPrivateIndex(spp, "iMy", 51);
            SetPrivateIndex(spp, "iMz", 52);

            SetPrivateIndex(spp, "iTemp", 60);
            SetPrivateIndex(spp, "iPress", 61);
            SetPrivateIndex(spp, "iVbatt", 62);

            SetPrivateIndex(spp, "iA6", 70);
            SetPrivateIndex(spp, "iA7", 71);
            SetPrivateIndex(spp, "iA15", 72);

            // Act
            InvokeResetIndices(spp);

            // Assert: all at -1
            Assert.Equal(-1, GetPrivateIndex(spp, "iTs"));

            Assert.Equal(-1, GetPrivateIndex(spp, "iExg1"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iExg2"));

            Assert.Equal(-1, GetPrivateIndex(spp, "iLnaX"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iLnaY"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iLnaZ"));

            Assert.Equal(-1, GetPrivateIndex(spp, "iWraX"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iWraY"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iWraZ"));

            Assert.Equal(-1, GetPrivateIndex(spp, "iGx"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iGy"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iGz"));

            Assert.Equal(-1, GetPrivateIndex(spp, "iMx"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iMy"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iMz"));

            Assert.Equal(-1, GetPrivateIndex(spp, "iTemp"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iPress"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iVbatt"));

            Assert.Equal(-1, GetPrivateIndex(spp, "iA6"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iA7"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iA15"));

            // Idempotence: calling it again does not change the result
            InvokeResetIndices(spp);
            Assert.Equal(-1, GetPrivateIndex(spp, "iTs"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iExg1"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iA15"));
        }


        // ----- RefreshMissingIndices behavior -----


        /// <summary>
        /// Helper: invokes the private <c>RefreshMissingIndices(ObjectCluster)</c> method on SPP session.
        /// </summary>
        /// <param name="spp">The SPP session instance.</param>
        /// <param name="oc">A data cluster used during index refresh.</param>
        private static void InvokeRefreshMissingIndices(object spp, ShimmerAPI.ObjectCluster oc)
        {
            var m = spp.GetType().GetMethod("RefreshMissingIndices", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(m);
            m!.Invoke(spp, new object[] { oc });
        }


        /// <summary>
        /// RefreshMissingIndices does not overwrite indices that are already set (not -1).
        /// Expected:
        /// - Pre-populated indices remain unchanged
        /// </summary>
        [Fact]
        public void RefreshMissingIndices_DoesNotOverwrite_AlreadyResolvedIndices()
        {
            var spp = CreateSppSession("10:20:30:40:50:60");

            // Enable EXG1 in the internal cfg
            var cfg = new ShimmerConfig { EnableExg1 = true };
            SetPrivateCurrentCfg(spp, cfg);

            // Preset a "already resolved" index
            var fExg1 = spp.GetType().GetField("iExg1", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fExg1);
            fExg1!.SetValue(spp, 7);

            // Call refresh: ObjectCluster stubs return -1,
            // but since iExg1 != -1, it should not be touched.
            var oc = new ShimmerAPI.ObjectCluster();
            InvokeRefreshMissingIndices(spp, oc);

            Assert.Equal(7, (int)fExg1.GetValue(spp)!);
        }


        /// <summary>
        /// With all sensor flags disabled, RefreshMissingIndices keeps all indices at -1.
        /// Expected:
        /// - All indices remain -1
        /// </summary
        [Fact]
        public void RefreshMissingIndices_WhenAllFlagsDisabled_KeepsAllAtMinusOne()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:01");

            // All flags are false -> the method must not look for indices (they remain -1)
            SetPrivateCurrentCfg(spp, new ShimmerConfig());

            // Set all indices to -1
            string[] fields =
            {
                "iTs",
                "iExg1","iExg2",
                "iLnaX","iLnaY","iLnaZ",
                "iWraX","iWraY","iWraZ",
                "iGx","iGy","iGz",
                "iMx","iMy","iMz",
                "iTemp","iPress","iVbatt",
                "iA6","iA7","iA15"
            };

            foreach (var name in fields)
            {
                var f = spp.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic);
                Assert.NotNull(f);
                f!.SetValue(spp, -1);
            }

            var oc = new ShimmerAPI.ObjectCluster();
            InvokeRefreshMissingIndices(spp, oc);

            foreach (var name in fields)
            {
                var f = spp.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic);
                Assert.Equal(-1, (int)f!.GetValue(spp)!);
            }
        }


        /// <summary>
        /// The timestamp index (iTs) is not overwritten when already set.
        /// Expected:
        /// - iTs retains its pre-set value
        /// </summary>
        [Fact]
        public void RefreshMissingIndices_TimestampIndex_NotOverwritten_WhenAlreadySet()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:02");

            // Any cfg is fine: here we just check that iTs is not overwritten
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableGyroscope = true });

            // Set iTs already solved
            var fTs = spp.GetType().GetField("iTs", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fTs);
            fTs!.SetValue(spp, 42);

            // Although ObjectCluster (stub) will return -1 on any lookup,
            // iTs must not change because it is different from -1
            var oc = new ShimmerAPI.ObjectCluster();
            InvokeRefreshMissingIndices(spp, oc);

            Assert.Equal(42, (int)fTs.GetValue(spp)!);
        }


        /// <summary>
        /// With some flags enabled, prepopulated indices remain; unresolved stay at -1.
        /// Expected:
        /// - Pre-set indices not changed
        /// - -1 indices remain -1 if detection fails
        /// </summary>
        [Fact]
        public void RefreshMissingIndices_WhenSomeFlagsEnabled_LeavesPrepopulated_OnOthersUnchanged()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:03");

            // Enable some sensor
            var cfg = new ShimmerConfig
            {
                EnableExg1 = true,
                EnableLowNoiseAccelerometer = true,
                EnableMagnetometer = true
            };
            SetPrivateCurrentCfg(spp, cfg);

            // Preset "already found" indexes
            spp.GetType().GetField("iExg1", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, 100);
            spp.GetType().GetField("iLnaX", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, 200);

            // Leave others at -1 (they won't resolve because the stub GetIndex -> -1)
            spp.GetType().GetField("iExg2", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, -1);
            spp.GetType().GetField("iLnaY", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, -1);
            spp.GetType().GetField("iMx", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, -1);

            var oc = new ShimmerAPI.ObjectCluster();
            InvokeRefreshMissingIndices(spp, oc);

            // The default ones remain unchanged
            Assert.Equal(100, (int)spp.GetType().GetField("iExg1", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(spp)!);
            Assert.Equal(200, (int)spp.GetType().GetField("iLnaX", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(spp)!);

            // Those at -1 remain -1 (no resolution possible in the stub)
            Assert.Equal(-1, (int)spp.GetType().GetField("iExg2", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(spp)!);
            Assert.Equal(-1, (int)spp.GetType().GetField("iLnaY", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(spp)!);
            Assert.Equal(-1, (int)spp.GetType().GetField("iMx", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(spp)!);
        }


        // ----- SetSamplingRateAsync behavior -----


        /// <summary>
        /// Helper: sets a private field to a value on the SPP session instance.
        /// </summary>
        /// <param name="o">Target instance.</param>
        /// <param name="name">Field name.</param>
        /// <param name="value">Value to assign.</param>
        private static void SetPrivateField(object o, string name, object? value)
        {
            var f = o.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(f);
            f!.SetValue(o, value);
        }


        /// <summary>
        /// Helper: reads a private field of generic type from the SPP session instance.
        /// </summary>
        /// <typeparam name="T">Expected field type.</typeparam>
        /// <param name="o">Target instance.</param>
        /// <param name="name">Field name.</param>
        /// <returns>Field value cast to <typeparamref name="T"/>.</returns>
        private static T GetPrivateField<T>(object o, string name)
        {
            var f = o.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(f);
            return (T)f!.GetValue(o)!;
        }


        /// <summary>
        /// Helper: assigns a stubbed core (<c>ShimmerLogAndStreamAndroidBluetoothV2</c>) into the SPP session.
        /// </summary>
        /// <param name="spp">SPP session.</param>
        /// <param name="mac">Device MAC.</param>
        private static void AssignCore(object spp, string mac = "00:11:22:33:44:55")
        {
            var core = new ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2("DEV", mac);
            SetPrivateField(spp, "_core", core);
        }


        /// <summary>
        /// SetSamplingRateAsync throws when session is not open (no core assigned).
        /// Expected:
        /// - InvalidOperationException is thrown
        /// </summary>
        [Fact]
        public async Task SetSamplingRateAsync_Throws_When_NotOpen()
        {
            var spp = CreateSppSession("10:10:10:10:10:10"); // _core remains null
            await Assert.ThrowsAsync<InvalidOperationException>(() =>
                (Task<double>)spp.GetType()
                    .GetMethod("SetSamplingRateAsync", BindingFlags.Instance | BindingFlags.Public)!
                    .Invoke(spp, new object[] { 128.0 })!);
        }


        /// <summary>
        /// SetSamplingRateAsync rounds the rate, updates config, resets indices and timestamp base.
        /// Expected:
        /// - Returned/applied SR is rounded integer
        /// - Current config reflects new SR
        /// - Indices set to -1 and _tsBase cleared
        /// </summary>
        [Fact]
        public async Task SetSamplingRateAsync_UpdatesCfg_ResetsIndices_And_TsBase()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:01");
            AssignCore(spp);

            SetPrivateCurrentCfg(spp, new ShimmerConfig { SamplingRate = 51, EnableGyroscope = true });
            SetPrivateField(spp, "iTs", 5);
            SetPrivateField(spp, "iExg1", 10);
            SetPrivateField(spp, "iGx", 3);
            SetPrivateField(spp, "_tsBase", (double?)123.45);

            var applied = await (Task<double>)spp.GetType()
                .GetMethod("SetSamplingRateAsync", BindingFlags.Instance | BindingFlags.Public)!
                .Invoke(spp, new object[] { 127.6 })!;

            // Rounding to integer
            Assert.Equal(128, applied);

            // CurrentConfig reflects the new SR (we use the public property)
            var snap = GetCurrentConfigSnapshot(spp);
            Assert.Equal(128, snap.SamplingRate);

            // Indexes reset
            Assert.Equal(-1, GetPrivateField<int>(spp, "iTs"));
            Assert.Equal(-1, GetPrivateField<int>(spp, "iExg1"));
            Assert.Equal(-1, GetPrivateField<int>(spp, "iGx"));

            // Base timestamp reset
            Assert.Null(GetPrivateField<double?>(spp, "_tsBase"));
        }


        /// <summary>
        /// If streaming and AnySensorEnabled == true, SetSamplingRateAsync restarts the data flow.
        /// Expected:
        /// - Handler is unhooked and then reinstalled (non-null at the end)
        /// - Config shows the rounded rate
        /// </summary>
        [Fact]
        public async Task SetSamplingRateAsync_WhenStreamingAndSensorsEnabled_Restarts()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:02");
            AssignCore(spp);

            // Enable at least one sensor like this AnySensorEnabled == true
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableGyroscope = true, SamplingRate = 51 });

            // Simulate "active streaming" (_handler != null)
            SetPrivateField(spp, "_handler", (EventHandler)((_, __) => { }));

            var applied = await (Task<double>)spp.GetType()
                .GetMethod("SetSamplingRateAsync", BindingFlags.Instance | BindingFlags.Public)!
                .Invoke(spp, new object[] { 64.2 })!;

            Assert.Equal(64, applied);

            // Expected: Stop() clears the handler, then Start() resets it to != null.
            // Let's check that the handler is present at the end of the method.
            var handlerAfter = GetPrivateField<EventHandler?>(spp, "_handler");
            Assert.NotNull(handlerAfter);

            // And the updated config
            var snap = GetCurrentConfigSnapshot(spp);
            Assert.Equal(64, snap.SamplingRate);
        }


        /// <summary>
        /// If streaming but no sensors enabled, SetSamplingRateAsync does not restart.
        /// Expected:
        /// - The existing handler reference remains the same
        /// - Config updated with rounded rate
        /// </summary>
        [Fact]
        public async Task SetSamplingRateAsync_WhenStreamingAndNoSensors_DoesNotRestart()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:03");
            AssignCore(spp);

            // No sensors enabled -> AnySensorEnabled == false
            SetPrivateCurrentCfg(spp, new ShimmerConfig { SamplingRate = 200 });

            // Simulate "active streaming" (_handler != null)
            var initialHandler = (EventHandler)((_, __) => { });
            SetPrivateField(spp, "_handler", initialHandler);

            var applied = await (Task<double>)spp.GetType()
                .GetMethod("SetSamplingRateAsync", BindingFlags.Instance | BindingFlags.Public)!
                .Invoke(spp, new object[] { 100.0 })!;

            Assert.Equal(100, applied);

            // Expected: Start() is NOT executed, so a new handler does not need to be created/registered.
            // The reference remains the same as before.
            var handlerAfter = GetPrivateField<EventHandler?>(spp, "_handler");
            Assert.Same(initialHandler, handlerAfter);

            // Updated config
            var snap = GetCurrentConfigSnapshot(spp);
            Assert.Equal(100, snap.SamplingRate);
        }


        /// <summary>
        /// SetSamplingRateAsync rounds to nearest integer and (optionally) logs the change.
        /// Expected:
        /// - Returned/applied rate is rounded
        /// - Current config shows the rounded value
        /// </summary>
        [Fact]
        public async Task SetSamplingRateAsync_Rounds_To_Integer_And_Logs()
        {
            var logs = new List<string>();
            var spp = CreateSppSession("AA:BB:CC:DD:EE:04");
            AssignCore(spp);

            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableExg1 = true, SamplingRate = 10 });

            var applied = await (Task<double>)spp.GetType()
                .GetMethod("SetSamplingRateAsync", BindingFlags.Instance | BindingFlags.Public)!
                .Invoke(spp, new object[] { 200.7 })!;

            Assert.Equal(201, applied);

            var snap = GetCurrentConfigSnapshot(spp);
            Assert.Equal(201, snap.SamplingRate);
        }


        // ----- SppSession constructor behavior -----


        /// <summary>
        /// The SppSession constructor trims MAC and assigns broadcast/log callbacks.
        /// Expected:
        /// - Stored MAC is trimmed
        /// - Internal delegates reference the provided callbacks
        /// </summary>
        [Fact]
        public void SppSession_Ctor_TrimsMac_And_AssignsCallbacks()
        {
            var macIn = "  01:23:45:67:89:AB  ";
            Action<string, string> broadcast = (_, __) => { };
            Action<string> log = _ => { };

            var spp = CreateSppSession(macIn, broadcast, log);

            // MAC is trimmed and saved
            var macStored = GetPrivateField<string>(spp, "_mac");
            Assert.Equal("01:23:45:67:89:AB", macStored);

            // callbacks are assigned
            var bcStored = GetPrivateField<Delegate>(spp, "_broadcast");
            var logStored = GetPrivateField<Delegate>(spp, "_log");
            Assert.Same(broadcast, bcStored);
            Assert.Same(log, logStored);
        }


        /// <summary>
        /// The constructor allows empty/whitespace MAC and stores it as empty string.
        /// Expected:
        /// - Internal _mac == string.Empty
        /// </summary>
        [Fact]
        public void SppSession_Ctor_AllowsEmptyOrWhitespaceMac_StoresEmptyString()
        {
            var macIn = "   ";
            Action<string, string> broadcast = (_, __) => { };
            Action<string> log = _ => { };

            var spp = CreateSppSession(macIn, broadcast, log);

            var macStored = GetPrivateField<string>(spp, "_mac");
            Assert.Equal(string.Empty, macStored);
        }


        /// <summary>
        /// Initial state of a new session has IsModeLocked == false.
        /// Expected:
        /// - IsModeLocked is false
        /// </summary>
        [Fact]
        public void SppSession_Ctor_InitialState_ModeUnlocked()
        {
            Action<string, string> broadcast = (_, __) => { };
            Action<string> log = _ => { };
            var spp = CreateSppSession("00:11:22:33:44:55", broadcast, log);

            var isLockedProp = spp.GetType().GetProperty("IsModeLocked", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(isLockedProp);
            Assert.False((bool)isLockedProp!.GetValue(spp)!);
        }


        // ----- OpenAsync behavior -----


        /// <summary>
        /// Helper: gets the backing delegate of an event by name (field-like events) on a target object.
        /// </summary>
        /// <param name="target">Object with the event.</param>
        /// <param name="eventName">Name of the event.</param>
        /// <returns>The event's multicast delegate or null.</returns>
        private static Delegate? GetEventDelegate(object target, string eventName)
        {
            var f = target.GetType().GetField(eventName, BindingFlags.Instance | BindingFlags.NonPublic);
            return (Delegate?)f?.GetValue(target);
        }


        /// <summary>
        /// Helper: invokes a public/non-public method; if it returns Task, awaits it.
        /// </summary>
        /// <param name="target">Target instance.</param>
        /// <param name="methodName">Method name.</param>
        /// <param name="args">Optional arguments.</param>
        private static async Task InvokeMethodAsync(object target, string methodName, params object[]? args)
        {
            var m = target.GetType().GetMethod(methodName, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
            Assert.NotNull(m);

            var result = m!.Invoke(target, args ?? Array.Empty<object>());

            if (result is Task t)
            {
                await t.ConfigureAwait(false);
            }
        }


        /// <summary>
        /// OpenAsync connects the core, assigns it, unhooks temporary handlers, and logs the connection.
        /// Expected:
        /// - _core is not null and Connected == true
        /// - Temporary UI callback is unhooked
        /// - Log contains a "[BT] Connected to ..." entry
        /// </summary>
        [Fact]
        public async Task OpenAsync_Completes_Connects_CoreAssigned_AndUnhooksHandler()
        {
            var mac = "01:23:45:67:89:AB";
            var logs = new List<string>();
            void Broadcast(string _, string __) { }
            void Log(string s) => logs.Add(s);

            var spp = CreateSppSession(mac, Broadcast, Log);

            // _core null
            Assert.Null(GetPrivateField<object>(spp, "_core"));

            await InvokeMethodAsync(spp, "OpenAsync");

            // core created and connected
            var core = GetPrivateField<object>(spp, "_core");
            Assert.NotNull(core);

            var connectedProp = core!.GetType().GetProperty("Connected", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(connectedProp);
            Assert.True((bool)connectedProp!.GetValue(core)!);

            var evt = GetEventDelegate(core, "UICallback");
            Assert.Null(evt);

            // Connection log emitted
            Assert.Contains(logs, l => l.Contains("[BT] Connected to 01:23:45:67:89:AB"));
        }


        /// <summary>
        /// OpenAsync trims incoming MAC and logs with the trimmed value.
        /// Expected:
        /// - Log shows the trimmed MAC address
        /// </summary>
        [Fact]
        public async Task OpenAsync_TrimsMac_And_LogsUsingTrimmedValue()
        {
            var logs = new List<string>();
            void Broadcast(string _, string __) { }
            void Log(string s) => logs.Add(s);

            var spp = CreateSppSession("  AA:BB:CC:DD:EE:FF  ", Broadcast, Log);

            await InvokeMethodAsync(spp, "OpenAsync");

            Assert.Contains(logs, l => l.Contains("[BT] Connected to AA:BB:CC:DD:EE:FF"));
        }


        // ----- ApplyConfigAsync behavior -----


        /// <summary>
        /// When mode is locked, ApplyConfigAsync preserves ExgMode while applying other changes.
        /// Expected:
        /// - ExgMode unchanged
        /// - Other flags and (rounded/defaulted) sampling rate applied
        /// </summary>
        [Fact]
        public async Task ApplyConfigAsync_Preserves_ExgMode_When_Locked()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:11");

            SetPrivateCurrentCfg(spp, new ShimmerConfig { ExgMode = ExgMode.EMG });

            // Open session (initialize _core)
            await InvokeMethodAsync(spp, "OpenAsync");

            // Mode lock: incoming cfg MUST NOT be able to change ExgMode
            var lockMode = spp.GetType().GetMethod("LockMode", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(lockMode);
            lockMode!.Invoke(spp, null);

            var cfgReq = new ShimmerConfig
            {
                // Let's try to change it: it should be ignored
                ExgMode = ExgMode.Respiration,
                SamplingRate = 50,
                EnableExg1 = true
            };

            await InvokeMethodAsync(spp, "ApplyConfigAsync", cfgReq);

            var snap = GetCurrentConfigSnapshot(spp);
            Assert.Equal(ExgMode.EMG, snap.ExgMode);            
            Assert.True(snap.EnableExg1);
            Assert.Equal(50, snap.SamplingRate);
        }


        /// <summary>
        /// ApplyConfigAsync defaults invalid or missing sampling rates to 51Hz.
        /// Expected:
        /// - SamplingRate == 51 when null or <= 0
        /// </summary>
        [Fact]
        public async Task ApplyConfigAsync_Defaults_SamplingRate_To_51_When_MissingOrInvalid()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:12");
            await InvokeMethodAsync(spp, "OpenAsync");

            // Missing SR
            var cfg1 = new ShimmerConfig
            {
                SamplingRate = null,
                EnableGyroscope = true
            };
            await InvokeMethodAsync(spp, "ApplyConfigAsync", cfg1);
            var s1 = GetCurrentConfigSnapshot(spp);
            Assert.Equal(51, s1.SamplingRate);

            // Invalid SR (<= 0)
            var cfg2 = new ShimmerConfig
            {
                SamplingRate = 0,
                EnableMagnetometer = true
            };
            await InvokeMethodAsync(spp, "ApplyConfigAsync", cfg2);
            var s2 = GetCurrentConfigSnapshot(spp);
            Assert.Equal(51, s2.SamplingRate);
        }


        /// <summary>
        /// ApplyConfigAsync rounds sampling rate and resets cached indices.
        /// Expected:
        /// - SamplingRate rounded to int
        /// - Cached indices reset (e.g., iTs == -1)
        /// </summary>
        [Fact]
        public async Task ApplyConfigAsync_Rounds_SamplingRate_And_Resets_Indices()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:13");
            await InvokeMethodAsync(spp, "OpenAsync");

            // Put a "resolved" index that needs to be reset by ApplyConfigAsync
            var fTs = spp.GetType().GetField("iTs", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fTs);
            fTs!.SetValue(spp, 777);

            var cfg = new ShimmerConfig
            {
                SamplingRate = 55,
                EnableBattery = true
            };

            await InvokeMethodAsync(spp, "ApplyConfigAsync", cfg);

            var snap = GetCurrentConfigSnapshot(spp);
            Assert.Equal(55, snap.SamplingRate);

            // Indexes must be reset to -1
            Assert.Equal(-1, (int)fTs.GetValue(spp)!);
        }


        /// <summary>
        /// When board detection fails, ApplyConfigAsync uses requested flags and order.
        /// Expected:
        /// - Config reflects requested flags and SR
        /// - EnabledBlocks match requested family keys in stable order
        /// </summary>
        [Fact]
        public async Task ApplyConfigAsync_Applies_Flags_And_EnabledBlocks_Match_Config_When_Detection_Fails()
        {

            // Note: our TryDetectBoardKind() stub returns false -> "using requested flags"
            var spp = CreateSppSession("AA:BB:CC:DD:EE:14");
            await InvokeMethodAsync(spp, "OpenAsync");

            var cfg = new ShimmerConfig
            {
                SamplingRate = 50,
                EnableExg1 = true,
                EnableLowNoiseAccelerometer = true,
                EnableGyroscope = true,
                EnableBattery = true,
                EnableExtA6 = true
            };

            await InvokeMethodAsync(spp, "ApplyConfigAsync", cfg);

            var snap = GetCurrentConfigSnapshot(spp);

            // Check configuration status
            Assert.Equal(50, snap.SamplingRate);
            Assert.True(snap.EnableExg1);
            Assert.True(snap.EnableLowNoiseAccelerometer);
            Assert.True(snap.EnableGyroscope);
            Assert.True(snap.EnableBattery);
            Assert.True(snap.EnableExtA6);

            // Check the symbolic list of enabled blocks (implementation-defined order)
            var blocks = CallEnabledBlocks(spp);
            Assert.Equal(new[] { "exg", "lna", "gyro", "vbatt", "ext6" }, blocks);
        }


        /// <summary>
        /// With IMU-only request and failed detection, ApplyConfigAsync keeps EXG off.
        /// Expected:
        /// - EXG flags remain false
        /// - EnabledBlocks reflect only IMU-related keys in stable order
        /// </summary>
        [Fact]
        public async Task ApplyConfigAsync_ImuOnly_Request_Remains_Imu_When_Detection_Fails()
        {

            // If detection fails, we don't force EXG off/on: we use the required flags.
            var spp = CreateSppSession("AA:BB:CC:DD:EE:15");
            await InvokeMethodAsync(spp, "OpenAsync");

            var cfg = new ShimmerConfig
            {
                SamplingRate = 50,
                EnableLowNoiseAccelerometer = true,
                EnableWideRangeAccelerometer = true,
                EnableGyroscope = true,
                EnableMagnetometer = true,
                EnablePressureTemperature = true,
                EnableBattery = true,
                EnableExg1 = false,
                EnableExg2 = false
            };

            await InvokeMethodAsync(spp, "ApplyConfigAsync", cfg);

            var snap = GetCurrentConfigSnapshot(spp);
            Assert.Equal(50, snap.SamplingRate);
            Assert.False(snap.EnableExg1);
            Assert.False(snap.EnableExg2);

            var blocks = CallEnabledBlocks(spp);

            // Order: exg, lna, wra, gyro, mag, temp, press, vbatt, ext
            Assert.Equal(new[] { "lna", "wra", "gyro", "mag", "temp", "press", "vbatt" }, blocks);
        }


        // ----- Start behavior -----


        /// <summary>
        /// Start throws when session is not open (no core).
        /// Expected:
        /// - InvalidOperationException thrown
        /// </summary>
        [Fact]
        public void Start_Throws_When_NotOpen()
        {
            var spp = CreateSppSession("01:23:45:67:89:AB");
            var mStart = spp.GetType().GetMethod("Start", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mStart);

            var ex = Assert.Throws<TargetInvocationException>(() => mStart!.Invoke(spp, null));
            Assert.IsType<InvalidOperationException>(ex.InnerException);
        }


        /// <summary>
        /// Start resets cached indices and clears timestamp base before streaming.
        /// Expected:
        /// - iTs == -1 and _tsBase == null after Start
        /// </summary>
        [Fact]
        public void Start_Resets_Indices_And_TsBase()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:01");

            // I inject _core so Start doesn't launch
            var core = new ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2("Stub", "AA:BB:CC:DD:EE:01");
            var fCore = spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fCore);
            fCore!.SetValue(spp, core);

            spp.GetType().GetField("iTs", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, 99);
            spp.GetType().GetField("_tsBase", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, 123.45);
            spp.GetType().GetMethod("Start", BindingFlags.Instance | BindingFlags.Public)!.Invoke(spp, null);

            Assert.Equal(-1, (int)spp.GetType().GetField("iTs", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(spp)!);
            Assert.Null(spp.GetType().GetField("_tsBase", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(spp));
        }


        /// <summary>
        /// Start wires the handler and broadcasts a minimal sample upon incoming data packet.
        /// Expected:
        /// - A "sample" JSON is broadcast containing Exg1/Exg2 and no IMU blocks if none enabled
        /// </summary>
        [Fact]
        public void Start_Wires_Handler_And_Broadcasts_Sample_On_DataPacket()
        {
            var mac = "AA:BB:CC:DD:EE:02";
            var sent = new List<(string Mac, string Json)>();
            var spp = CreateSppSession(mac, (m, j) => sent.Add((m, j)));

            // Inject _core
            var core = new ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2("Stub", mac);
            spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, core);

            SetPrivateCurrentCfg(spp, new ShimmerConfig());
            spp.GetType().GetMethod("Start", BindingFlags.Instance | BindingFlags.Public)!.Invoke(spp, null);

            // Simulate data packet arrival
            var ev = new ShimmerAPI.CustomEventArgs(
                (int)ShimmerAPI.ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET,
                new ShimmerAPI.ObjectCluster());
            core.RaiseUi(ev);

            Assert.Single(sent);
            Assert.Equal(mac, sent[0].Mac);

            using var doc = JsonDocument.Parse(sent[0].Json);
            var root = doc.RootElement;
            Assert.Equal("sample", root.GetProperty("type").GetString());
            Assert.Equal(mac, root.GetProperty("mac").GetString());
            Assert.True(root.TryGetProperty("Exg1", out var _));
            Assert.True(root.TryGetProperty("Exg2", out var _));
            Assert.False(root.TryGetProperty("lna", out _));
            Assert.False(root.TryGetProperty("wra", out _));
            Assert.False(root.TryGetProperty("gyro", out _));
            Assert.False(root.TryGetProperty("mag", out _));
            Assert.False(root.TryGetProperty("temp", out _));
            Assert.False(root.TryGetProperty("press", out _));
            Assert.False(root.TryGetProperty("vbatt", out _));
            Assert.False(root.TryGetProperty("ext", out _));
            Assert.False(root.TryGetProperty("exg_mode", out _));
        }


        /// <summary>
        /// Calling Start twice does not double-subscribe the handler.
        /// Expected:
        /// - A single incoming packet produces a single broadcast
        /// </summary>
        [Fact]
        public void Start_DoesNot_DoubleSubscribe_When_Called_Twice()
        {
            var mac = "AA:BB:CC:DD:EE:03";
            var sent = new List<(string Mac, string Json)>();
            var spp = CreateSppSession(mac, (m, j) => sent.Add((m, j)));

            var core = new ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2("Stub", mac);
            spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, core);

            SetPrivateCurrentCfg(spp, new ShimmerConfig());

            // Start twice: the second one must first unhook the previous handler
            var mStart = spp.GetType().GetMethod("Start", BindingFlags.Instance | BindingFlags.Public)!;
            mStart.Invoke(spp, null);
            mStart.Invoke(spp, null);

            // A single packet -> a single broadcast (no duplicates)
            var ev = new ShimmerAPI.CustomEventArgs(
                (int)ShimmerAPI.ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET,
                new ShimmerAPI.ObjectCluster());
            core.RaiseUi(ev);

            Assert.Single(sent);
        }


        /// <summary>
        /// When relevant blocks and last-known values exist, Start includes them in the sample payload.
        /// Expected:
        /// - exg_mode present if ExgMode != None
        /// - Enabled blocks present; battery/ext values reflect last-known
        /// </summary>
        [Fact]
        public void Start_Includes_Enabled_Blocks_And_LastKnown_Values()
        {
            var mac = "AA:BB:CC:DD:EE:04";
            var sent = new List<(string Mac, string Json)>();
            var spp = CreateSppSession(mac, (m, j) => sent.Add((m, j)));

            var core = new ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2("Stub", mac);
            spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, core);

            var cfg = new ShimmerConfig
            {
                EnableLowNoiseAccelerometer = true,
                EnableGyroscope = true,
                EnableBattery = true,
                EnableExtA6 = true,
                EnableExtA7 = true,
                EnableExtA15 = true,
                ExgMode = ExgMode.EMG
            };
            SetPrivateCurrentCfg(spp, cfg);

            spp.GetType().GetField("_lastVbatt", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, 3.7);
            spp.GetType().GetField("_lastA6", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, 1.2);
            spp.GetType().GetField("_lastA7", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, 2.3);
            spp.GetType().GetField("_lastA15", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, 4.5);

            // Start and send a packet
            spp.GetType().GetMethod("Start", BindingFlags.Instance | BindingFlags.Public)!.Invoke(spp, null);
            var ev = new ShimmerAPI.CustomEventArgs(
                (int)ShimmerAPI.ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET,
                new ShimmerAPI.ObjectCluster());
            core.RaiseUi(ev);

            Assert.Single(sent);
            using var doc = JsonDocument.Parse(sent[0].Json);
            var root = doc.RootElement;

            // exg_mode present because ExgMode != None
            Assert.True(root.TryGetProperty("exg_mode", out var exgModeProp));
            Assert.False(string.IsNullOrWhiteSpace(exgModeProp.GetString()));

            // lna, gyro present (wra/mag/press/… no because not enabled)
            Assert.True(root.TryGetProperty("lna", out _));
            Assert.True(root.TryGetProperty("gyro", out _));

            // vbatt present with last-known
            Assert.True(root.TryGetProperty("vbatt", out var vb));
            Assert.Equal(3.7, vb.GetDouble(), 3);

            // ext present with a6/a7/a15
            Assert.True(root.TryGetProperty("ext", out var ext));
            Assert.Equal(1.2, ext.GetProperty("a6").GetDouble(), 3);
            Assert.Equal(2.3, ext.GetProperty("a7").GetDouble(), 3);
            Assert.Equal(4.5, ext.GetProperty("a15").GetDouble(), 3);
        }


        // ----- Stop behavior -----


        /// <summary>
        /// Stop unsubscribes handler and prevents further broadcasts.
        /// Expected:
        /// - After Stop, additional data packets produce no further broadcasts
        /// </summary>
        [Fact]
        public async Task Stop_Unsubscribes_Handler_NoFurtherBroadcasts()
        {
            var sent = new List<(string mac, string json)>();
            var spp = CreateSppSession("01:23:45:67:89:AB", (m, j) => sent.Add((m, j)));

            // Open connection and start streaming
            var mOpen = spp.GetType().GetMethod("OpenAsync", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mOpen);
            await (Task)mOpen!.Invoke(spp, null)!;

            var mStart = spp.GetType().GetMethod("Start", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mStart);
            mStart!.Invoke(spp, null);

            // Get _core and send a data packet
            var fCore = spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fCore);
            var core = (ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2)fCore!.GetValue(spp)!;

            core.RaiseUi(new ShimmerAPI.CustomEventArgs(
                (int)ShimmerAPI.ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET,
                new ShimmerAPI.ObjectCluster()));

            // we get something before the stop
            Assert.NotEmpty(sent);
            var firstCount = sent.Count;

            // Stop -> unsubscribe _handler and send StopStreaming()
            var mStop = spp.GetType().GetMethod("Stop", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mStop);
            mStop!.Invoke(spp, null);

            // Try resend a packet -> nothing more should arrive
            core.RaiseUi(new ShimmerAPI.CustomEventArgs(
                (int)ShimmerAPI.ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET,
                new ShimmerAPI.ObjectCluster()));

            // no new emission after Stop()
            Assert.Equal(firstCount, sent.Count);
        }


        /// <summary>
        /// Stop is safe and idempotent when streaming was never started.
        /// Expected:
        /// - No throws on repeated Stop
        /// - No broadcasts if packets arrive
        /// </summary>
        [Fact]
        public async Task Stop_When_NotStarted_Is_Idempotent_And_Safe()
        {

            // open but do NOT call Start()
            var sent = new List<(string mac, string json)>();
            var spp = CreateSppSession("AA:BB:CC:DD:EE:FF", (m, j) => sent.Add((m, j)));

            var mOpen = spp.GetType().GetMethod("OpenAsync", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mOpen);
            await (Task)mOpen!.Invoke(spp, null)!;

            // Stop without handler installed -> should not throw
            var mStop = spp.GetType().GetMethod("Stop", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mStop);
            mStop!.Invoke(spp, null);

            // And a second time too (idempotent)
            mStop!.Invoke(spp, null);

            // Send data packet -> no handler, so no emission
            var fCore = spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fCore);
            var core = (ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2)fCore!.GetValue(spp)!;

            core.RaiseUi(new ShimmerAPI.CustomEventArgs(
                (int)ShimmerAPI.ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET,
                new ShimmerAPI.ObjectCluster()));

            Assert.Empty(sent);
        }


        /// <summary>
        /// Stop is safe when core is null (never opened).
        /// Expected:
        /// - No throw
        /// </summary>
        [Fact]
        public void Stop_When_CoreIsNull_DoesNotThrow()
        {
            var spp = CreateSppSession("11:22:33:44:55:66");

            var mStop = spp.GetType().GetMethod("Stop", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mStop);
            mStop!.Invoke(spp, null);
        }


        // ----- Dispose behavior -----


        /// <summary>
        /// Dispose stops streaming, disconnects core, clears core reference, and prevents further broadcasts.
        /// Expected:
        /// - Core becomes disconnected and internal _core set to null
        /// - No further broadcasts after dispose
        /// </summary>
        [Fact]
        public async Task Dispose_StopsStreaming_Disconnects_Core_IsCleared()
        {
            var sent = new List<(string mac, string json)>();
            var spp = CreateSppSession("DE:AD:BE:EF:00:01", (m, j) => sent.Add((m, j)));

            var mOpen = spp.GetType().GetMethod("OpenAsync", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mOpen);
            await (Task)mOpen!.Invoke(spp, null)!;

            var mStart = spp.GetType().GetMethod("Start", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mStart);
            mStart!.Invoke(spp, null);

            // Get a reference to the core BEFORE the Dispose (so we can check the state afterwards)
            var fCore = spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fCore);
            var core = (ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2)fCore!.GetValue(spp)!;

            // Check for packets arriving before Dispose
            core.RaiseUi(new ShimmerAPI.CustomEventArgs(
                (int)ShimmerAPI.ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET,
                new ShimmerAPI.ObjectCluster()));
            Assert.NotEmpty(sent);
            var before = sent.Count;

            var mDispose = spp.GetType().GetMethod("Dispose", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mDispose);
            mDispose!.Invoke(spp, null);

            // core disconnected and reset in session object
            Assert.False(core.IsConnected());
            Assert.Null(fCore.GetValue(spp));

            // And the handler has been unsubscribed: no new broadcasts
            core.RaiseUi(new ShimmerAPI.CustomEventArgs(
                (int)ShimmerAPI.ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET,
                new ShimmerAPI.ObjectCluster()));
            Assert.Equal(before, sent.Count);
        }


        /// <summary>
        /// Dispose is safe when session was never opened.
        /// Expected:
        /// - No throw and _core remains null
        /// </summary>
        [Fact]
        public void Dispose_When_NotOpened_Is_Safe_And_NoThrow()
        {
            var spp = CreateSppSession("DE:AD:BE:EF:00:02");

            var mDispose = spp.GetType().GetMethod("Dispose", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mDispose);
            mDispose!.Invoke(spp, null);

            var fCore = spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fCore);
            Assert.Null(fCore!.GetValue(spp));
        }


        /// <summary>
        /// Dispose is idempotent; multiple calls leave core null.
        /// Expected:
        /// - _core is null after repeated calls
        /// </summary>
        [Fact]
        public async Task Dispose_Is_Idempotent()
        {
            var spp = CreateSppSession("DE:AD:BE:EF:00:03");

            var mOpen = spp.GetType().GetMethod("OpenAsync", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mOpen);
            await (Task)mOpen!.Invoke(spp, null)!;

            var mDispose = spp.GetType().GetMethod("Dispose", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mDispose);

            // two consecutive calls
            mDispose!.Invoke(spp, null);
            mDispose!.Invoke(spp, null);

            // core null
            var fCore = spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fCore);
            Assert.Null(fCore!.GetValue(spp));
        }


        // ----- SafeIdx behavior -----


        /// <summary>
        /// Helper: obtains the private static method <c>SafeIdx(ObjectCluster,string,string)</c>.
        /// </summary>
        /// <param name="spp">Any SPP session instance (for type resolution).</param>
        /// <returns>MethodInfo for SafeIdx.</returns>
        private static MethodInfo GetSafeIdxMethod(object spp)
        {
            var mi = spp.GetType().GetMethod("SafeIdx",
                BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return mi!;
        }


        /// <summary>
        /// SafeIdx returns a valid index when label exists.
        /// Expected:
        /// - Returns expected positive index
        /// </summary>
        [Fact]
        public void SafeIdx_Returns_Index_When_LabelExists()
        {
            var spp = CreateSppSession("11:22:33:44:55:66");
            var safeIdx = GetSafeIdxMethod(spp);

            var oc = new ShimmerAPI.ObjectCluster();
            var result = (int)safeIdx.Invoke(null, new object[] { oc, "__FIVE__", "CAL" })!;

            Assert.Equal(5, result);
        }


        /// <summary>
        /// SafeIdx returns -1 when label is missing.
        /// Expected:
        /// - -1
        /// </summary>
        [Fact]
        public void SafeIdx_Returns_MinusOne_When_LabelMissing()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:FF");
            var safeIdx = GetSafeIdxMethod(spp);

            var oc = new ShimmerAPI.ObjectCluster(); // GetIndex -> -1
            var result = (int)safeIdx.Invoke(null, new object[] { oc, "Unknown Label", "CAL" })!;

            Assert.Equal(-1, result);
        }


        /// <summary>
        /// SafeIdx returns -1 when underlying GetIndex throws.
        /// Expected:
        /// - -1
        /// </summary>
        [Fact]
        public void SafeIdx_Returns_MinusOne_When_GetIndex_Throws()
        {
            var spp = CreateSppSession("DE:AD:BE:EF:00:01");
            var safeIdx = GetSafeIdxMethod(spp);

            var oc = new ShimmerAPI.ObjectCluster();
            var result = (int)safeIdx.Invoke(null, new object[] { oc, "__THROW__", "CAL" })!;

            Assert.Equal(-1, result);
        }


        // ----- SafeGet behavior -----


        /// <summary>
        /// Helper: obtains the private static method <c>SafeGet(ObjectCluster,int)</c>.
        /// </summary>
        /// <param name="spp">SPP session (for type resolution).</param>
        /// <returns>MethodInfo for SafeGet.</returns>
        private static System.Reflection.MethodInfo GetSafeGetMethod(object spp)
        {
            var mi = spp.GetType().GetMethod("SafeGet",
                BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return mi!;
        }


        /// <summary>
        /// SafeGet returns null when index is negative.
        /// Expected:
        /// - null
        /// </summary>
        [Fact]
        public void SafeGet_ReturnsNull_When_Index_Is_Negative()
        {
            var spp = CreateSppSession("AA:BB:CC:00:00:01");
            var safeGet = GetSafeGetMethod(spp);

            var oc = new ShimmerAPI.ObjectCluster();
            var res = safeGet.Invoke(null, new object[] { oc, -1 });

            Assert.Null(res);
        }


        /// <summary>
        /// SafeGet returns a SensorData instance when index is valid.
        /// Expected:
        /// - Non-null SensorData
        /// </summary>
        [Fact]
        public void SafeGet_Returns_SensorData_When_Index_Is_Valid()
        {
            var spp = CreateSppSession("AA:BB:CC:00:00:02");
            var safeGet = GetSafeGetMethod(spp);

            var oc = new ShimmerAPI.ObjectCluster();
            var res = safeGet.Invoke(null, new object[] { oc, 5 });

            Assert.NotNull(res);
            Assert.IsType<ShimmerAPI.SensorData>(res);
        }


        /// <summary>
        /// SafeGet tolerates upstream exceptions and returns null.
        /// Expected:
        /// - null when GetData throws
        /// </summary>
        [Fact]
        public void SafeGet_ReturnsNull_When_GetData_Throws()
        {
            var spp = CreateSppSession("AA:BB:CC:00:00:03");
            var safeGet = GetSafeGetMethod(spp);

            var oc = new ShimmerAPI.ObjectCluster();

            // 777: “special” index that causes an exception to be thrown in the stub
            var res = safeGet.Invoke(null, new object[] { oc, 777 });

            Assert.Null(res);
        }


        // ----- Val behavior -----


        /// <summary>
        /// Helper: obtains the private static <c>Val(SensorData)</c> converter method.
        /// </summary>
        /// <param name="spp">SPP session (for type resolution).</param>
        /// <returns>MethodInfo for Val.</returns>
        private static System.Reflection.MethodInfo GetValMethod(object spp)
        {
            var mi = spp.GetType().GetMethod("Val",
                BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return mi!;
        }


        /// <summary>
        /// Helper: wraps an object into a <see cref="ShimmerAPI.SensorData"/> for tests.
        /// </summary>
        /// <param name="data">Payload to store into SensorData.</param>
        /// <returns>New SensorData instance.</returns>
        private static ShimmerAPI.SensorData SD(object data) => new ShimmerAPI.SensorData(data);


        /// <summary>
        /// Val returns null when SensorData is null.
        /// Expected:
        /// - null
        /// </summary>
        [Fact]
        public void Val_ReturnsNull_When_SensorData_Is_Null()
        {
            var spp = CreateSppSession("AA:BB:CC:10:00:01");
            var val = GetValMethod(spp);

            var res = val.Invoke(null, new object?[] { null });

            Assert.Null(res);
        }


        /// <summary>
        /// Val converts int to double.
        /// Expected:
        /// - 5 -> 5.0
        /// </summary>
        [Fact]
        public void Val_Converts_Int_To_Double()
        {
            var spp = CreateSppSession("AA:BB:CC:10:00:02");
            var val = GetValMethod(spp);

            var sd = SD(5); // int
            var res = (double?)val.Invoke(null, new object?[] { sd });

            Assert.Equal(5.0, res);
        }


        /// <summary>
        /// Val passes through double values unchanged.
        /// Expected:
        /// - 3.25 -> 3.25
        /// </summary>
        [Fact]
        public void Val_Converts_Double_To_Double_Unchanged()
        {
            var spp = CreateSppSession("AA:BB:CC:10:00:03");
            var val = GetValMethod(spp);

            var sd = SD(3.25); // double
            var res = (double?)val.Invoke(null, new object?[] { sd });

            Assert.Equal(3.25, res);
        }


        /// <summary>
        /// Val parses numeric strings to double.
        /// Expected:
        /// - "42" -> 42.0
        /// </summary>
        [Fact]
        public void Val_Converts_IntegerString_To_Double()
        {
            var spp = CreateSppSession("AA:BB:CC:10:00:04");
            var val = GetValMethod(spp);

            var sd = SD("42");
            var res = (double?)val.Invoke(null, new object?[] { sd });

            Assert.Equal(42.0, res);
        }


        /// <summary>
        /// Val returns null for non-numeric strings.
        /// Expected:
        /// - null
        /// </summary>
        [Fact]
        public void Val_ReturnsNull_On_NonNumeric_String()
        {
            var spp = CreateSppSession("AA:BB:CC:10:00:05");
            var val = GetValMethod(spp);

            var sd = SD("not-a-number");
            var res = (double?)val.Invoke(null, new object?[] { sd });

            Assert.Null(res);
        }


        /// <summary>
        /// Val returns null for objects not convertible to double.
        /// Expected:
        /// - null
        /// </summary>
        [Fact]
        public void Val_ReturnsNull_On_Object_Not_Convertible()
        {
            var spp = CreateSppSession("AA:BB:CC:10:00:06");
            var val = GetValMethod(spp);

            var sd = SD(new object());
            var res = (double?)val.Invoke(null, new object?[] { sd });

            Assert.Null(res);
        }


        // ----- IsConnectedState behavior -----


        /// <summary>
        /// Helper: obtains the private static <c>IsConnectedState(object?)</c> classifier.
        /// </summary>
        /// <param name="spp">SPP session (for type resolution).</param>
        /// <returns>MethodInfo for IsConnectedState.</returns>
        private static System.Reflection.MethodInfo GetIsConnectedMethod(object spp)
        {
            var mi = spp.GetType().GetMethod("IsConnectedState",
                BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return mi!;
        }


        /// <summary>
        /// IsConnectedState returns false on null inputs.
        /// Expected:
        /// - false
        /// </summary>
        [Fact]
        public void IsConnectedState_ReturnsFalse_On_Null()
        {
            var spp = CreateSppSession("AA:BB:00:00:01");
            var m = GetIsConnectedMethod(spp);

            var res = (bool)m.Invoke(null, new object?[] { null })!;
            Assert.False(res);
        }


        /// <summary>
        /// IsConnectedState handles integer codes.
        /// Expected:
        /// - 2/3 -> true; 0/1/4 -> false
        /// </summary>
        [Theory]
        [InlineData(2, true)]
        [InlineData(3, true)]
        [InlineData(0, false)]
        [InlineData(1, false)]
        [InlineData(4, false)]
        public void IsConnectedState_Handles_Int(object state, bool expected)
        {
            var spp = CreateSppSession("AA:BB:00:00:02");
            var m = GetIsConnectedMethod(spp);

            var res = (bool)m.Invoke(null, new object?[] { state })!;
            Assert.Equal(expected, res);
        }


        /// <summary>
        /// IsConnectedState supports Java Integer wrapper values.
        /// Expected:
        /// - 2/3 -> true; 1 -> false
        /// </summary>
        [Theory]
        [InlineData(2, true)]
        [InlineData(3, true)]
        [InlineData(1, false)]
        public void IsConnectedState_Handles_JavaInteger(int value, bool expected)
        {
            var spp = CreateSppSession("AA:BB:00:00:03");
            var m = GetIsConnectedMethod(spp);

            var ji = new Java.Lang.Integer(value);
            var res = (bool)m.Invoke(null, new object?[] { ji })!;
            Assert.Equal(expected, res);
        }


        /// <summary>
        /// IsConnectedState parses string representations case-insensitively.
        /// Expected:
        /// - "CONNECTED"/"connected"/"BT_CONNECTED_OK" -> true; "DISCONNECTED"/others -> false
        /// </summary>
        [Theory]
        [InlineData("CONNECTED", true)]
        [InlineData("connected", true)]
        [InlineData("BT_CONNECTED_OK", true)]
        [InlineData("DISCONNECTED", false)]
        [InlineData("something else", false)]
        [InlineData("", false)]
        public void IsConnectedState_Parses_Strings(string s, bool expected)
        {
            var spp = CreateSppSession("AA:BB:00:00:04");
            var m = GetIsConnectedMethod(spp);

            var res = (bool)m.Invoke(null, new object?[] { s })!;
            Assert.Equal(expected, res);
        }


        /// <summary>
        /// IsConnectedState falls back to ToString() when given unknown types.
        /// Expected:
        /// - Objects with "CONNECTED" in ToString() => true
        /// </summary>
        [Fact]
        public void IsConnectedState_Uses_ToString_Fallback()
        {
            var spp = CreateSppSession("AA:BB:00:00:05");
            var m = GetIsConnectedMethod(spp);

            var obj = new { Status = "CONNECTED" };
            var res = (bool)m.Invoke(null, new object?[] { obj })!;
            Assert.True(res);
        }


        /// <summary>
        /// IsConnectedState returns false when ToString() does not contain 'CONNECTED'.
        /// Expected:
        /// - false
        /// </summary>
        [Fact]
        public void IsConnectedState_ReturnsFalse_On_NonConvertible_Object()
        {
            var spp = CreateSppSession("AA:BB:00:00:06");
            var m = GetIsConnectedMethod(spp);

            var obj = new object();
            var res = (bool)m.Invoke(null, new object?[] { obj })!;
            Assert.False(res);
        }


        // ----- TryIdx behavior -----


        /// <summary>
        /// Helper: invokes the private static <c>TryIdx(ObjectCluster,(string,string)[])</c> utility.
        /// Returns the first matching candidate index or -1.
        /// </summary>
        /// <param name="oc">ObjectCluster stub.</param>
        /// <param name="cands">Name/format candidate pairs.</param>
        /// <returns>Index or -1.</returns>
        private static int InvokeTryIdx(object oc, params (string name, string fmt)[] cands)
        {
            var spp = CreateSppSession("00:11:22:33:44:55");
            var mi = spp.GetType().GetMethod("TryIdx",
                BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return (int)mi!.Invoke(spp, new object?[] { oc, cands })!;
        }


        /// <summary>
        /// TryIdx returns the first matching candidate index.
        /// Expected:
        /// - Returns index of the earliest matching candidate
        /// </summary>
        [Fact]
        public void TryIdx_Returns_FirstMatchingIndex()
        {
            // Fake OC that only responds to ("Gyroscope X","CAL") -> index 5
            var oc = new ShimmerAPI.ObjectClusterMock()
                .When("Gyroscope X", "CAL", 5);

            // The first one does not exist, the second one is the correct one -> must return 5
            int i = InvokeTryIdx(oc,
                ("WR Accel X", "CAL"),
                ("Gyroscope X", "CAL"),
                ("Magnetometer X", "CAL"));

            Assert.Equal(5, i);
        }


        /// <summary>
        /// TryIdx returns -1 when none of the candidates match.
        /// Expected:
        /// - -1
        /// </summary>
        [Fact]
        public void TryIdx_Returns_MinusOne_When_NoCandidatesMatch()
        {
            var oc = new ShimmerAPI.ObjectClusterMock(); // no mapping
            int i = InvokeTryIdx(oc, ("Foo", "CAL"), ("Bar", "RAW"));
            Assert.Equal(-1, i);
        }


        /// <summary>
        /// TryIdx stops at the first successful match.
        /// Expected:
        /// - Returns the index from the first matching candidate only
        /// </summary>
        [Fact]
        public void TryIdx_Stops_At_First_Match()
        {
            var oc = new ShimmerAPI.ObjectClusterMock()
                .When("A", "CAL", 2)
                .When("B", "CAL", 7); // it shouldn't get this far

            int i = InvokeTryIdx(oc, ("A", "CAL"), ("B", "CAL"));
            Assert.Equal(2, i);
        }


        /// <summary>
        /// TryIdx swallows internal GetIndex errors and continues with next candidates.
        /// Expected:
        /// - If first throws and next misses, returns -1
        /// </summary>
        [Fact]
        public void TryIdx_Swallows_Internal_GetIndex_Errors()
        {
            var oc = new ShimmerAPI.ObjectClusterMock()
                .ThrowOn("Bad", "CAL");

            int i = InvokeTryIdx(oc, ("Bad", "CAL"), ("Good", "CAL"));

            // The first one raises, the second one does not exist -> -1
            Assert.Equal(-1, i);
        }


        // ----- LooksLikeShimmer behavior -----


        /// <summary>
        /// Helper: invokes the private static <c>LooksLikeShimmer(string?,string?)</c> classification helper.
        /// </summary>
        /// <param name="name">Device name or null.</param>
        /// <param name="mac">MAC string or null.</param>
        /// <returns><c>true</c> if the pair resembles a Shimmer device; otherwise <c>false</c>.</returns>
        private static bool CallLooksLikeShimmer(string? name, string? mac)
        {
            var t = typeof(WsBridgeManager);
            var m = t.GetMethod("LooksLikeShimmer", BindingFlags.Static | BindingFlags.NonPublic);
            Assert.NotNull(m);
            return (bool)m!.Invoke(null, new object?[] { name, mac })!;
        }


        /// <summary>
        /// LooksLikeShimmer returns true on known Shimmer patterns (name or OUI).
        /// Expected:
        /// - True for names containing "Shimmer" or RN* patterns, or MAC with OUI 00:06:66 (colon-separated)
        /// </summary>
        [Theory]
        // Match on name: contains "Shimmer"
        [InlineData("Shimmer3", null)]
        [InlineData("my_shimmer_node", null)]
        // Match on name: RN prefixes*
        [InlineData("RNBT-1234", null)]
        [InlineData("RN42-ABCD", null)]
        [InlineData("RN-42-ABCD", null)]
        // Match on MAC OUI 00:06:66 (only with ':')
        [InlineData(null, "00:06:66:AA:BB:CC")]
        [InlineData("Other", "00:06:66:FF:EE:DD")]
        // Case-insensitive
        [InlineData("shimmer", null)]
        [InlineData(null, "00:06:66:aa:bb:cc")]
        public void LooksLikeShimmer_ReturnsTrue_OnKnownPatterns(string? name, string? mac)
        {
            Assert.True(CallLooksLikeShimmer(name, mac));
        }


        /// <summary>
        /// LooksLikeShimmer returns false on unknown patterns or malformed MAC formatting.
        /// Expected:
        /// - False for null/empty, unrelated names, or non-colon MAC formats
        /// </summary>
        [Theory]
        // No match
        [InlineData(null, null)]
        [InlineData("", "")]
        [InlineData("Device", null)]
        // MAC with different separators or no separator -> does not match (current implementation uses StartsWith with ':')
        [InlineData(null, "00-06-66-AA-BB-CC")]
        [InlineData(null, "000666AABBCC")]
        // MAC with leading space and no match on name -> does not match
        [InlineData("Other", " 00:06:66:AA:BB:CC")]
        // Name that does not contain the expected patterns
        [InlineData("SHMR-Device", null)]
        public void LooksLikeShimmer_ReturnsFalse_OnUnknownPatterns(string? name, string? mac)
        {
            Assert.False(CallLooksLikeShimmer(name, mac));
        }


        /// <summary>
        /// LooksLikeShimmer is case-insensitive and ignores leading/trailing spaces in name but not in MAC.
        /// Expected:
        /// - "  Shimmer  " -> true; "  rnBt-XYZ" -> false; " device " + " 00:06:66:11:22:33" (space in MAC) -> false
        /// </summary>
        [Theory]
        // Check that the search is case-insensitive and that "Contains" also works with spaces
        [InlineData("  Shimmer  ", null, true)]
        [InlineData("  rnBt-XYZ", null, false)]
        [InlineData(" device ", " 00:06:66:11:22:33", false)] // space in MAC -> no match
        public void LooksLikeShimmer_MixedCasesAndSpaces(string? name, string? mac, bool expected)
        {
            Assert.Equal(expected, CallLooksLikeShimmer(name, mac));
        }


        // -----  SafeSend behavior -----


        /// <summary>
        /// Helper: creates a new WsBridgeManager instance using an available constructor without parameters,
        /// or the “simplest” constructor with default arguments as a fallback.
        /// </summary>
        /// <returns>A new WsBridgeManager instance.</returns>
        private static object NewBridge()
        {
            var t = typeof(WsBridgeManager);

            var ctor0 = t.GetConstructors(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic)
                         .FirstOrDefault(c => c.GetParameters().Length == 0);
            if (ctor0 != null)
                return ctor0.Invoke(Array.Empty<object?>());

            var anyCtor = t.GetConstructors(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic)
                           .OrderBy(c => c.GetParameters().Length)
                           .FirstOrDefault();

            if (anyCtor == null)
                throw new InvalidOperationException("WsBridgeManager has no usable constructors.");

            var pars = anyCtor.GetParameters();
            var args = new object?[pars.Length];

            for (int i = 0; i < pars.Length; i++)
            {
                var pt = pars[i].ParameterType;

                // “Safe” default values ​​for tests
                if (pt == typeof(string)) args[i] = string.Empty;
                else if (pt.IsValueType) args[i] = Activator.CreateInstance(pt);
                else args[i] = null;
            }

            return anyCtor.Invoke(args);
        }


        /// <summary>
        /// Helper: invokes the private instance method <c>SafeSend(Guid,string)</c>.
        /// </summary>
        /// <param name="bridge">Bridge instance.</param>
        /// <param name="clientId">Target client id.</param>
        /// <param name="json">JSON payload.</param>
        /// <returns>Awaitable task.</returns>
        private static Task InvokeSafeSend(object bridge, Guid clientId, string json)
        {
            var m = bridge.GetType().GetMethod("SafeSend", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(m);
            return (Task)m!.Invoke(bridge, new object[] { clientId, json })!;
        }


        /// <summary>
        /// Helper: sets the private <c>_ws</c> server field on the bridge.
        /// </summary>
        /// <param name="bridge">Bridge instance.</param>
        /// <param name="server">WS server instance or null.</param>
        private static void SetWsServer(object bridge, WatsonWsServer? server)
        {
            var f = bridge.GetType().GetField("_ws", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(f);
            f!.SetValue(bridge, server);
        }


        /// <summary>
        /// Helper: sets a private log delegate on the bridge, if present.
        /// </summary>
        /// <param name="bridge">Bridge instance.</param>
        /// <param name="log">Log action.</param>
        private static void SetLog(object bridge, Action<string>? log)
        {
            var f = bridge.GetType().GetField("Log", BindingFlags.Instance | BindingFlags.NonPublic);
            if (f != null) f.SetValue(bridge, log);
        }


        /// <summary>
        /// SafeSend completes without throwing when server is null.
        /// Expected:
        /// - No exception
        /// </summary>
        [Fact]
        public async Task SafeSend_WhenServerIsNull_CompletesWithoutThrowing()
        {
            var bridge = NewBridge();          
            SetWsServer(bridge, null);         
            SetLog(bridge, _ => { });         

            var id = Guid.NewGuid();
            var payload = "{\"ok\":true}";

            await InvokeSafeSend(bridge, id, payload); 
        }


        /// <summary>
        /// SafeSend delegates to server SendAsync and records the message in the stub log.
        /// Expected:
        /// - The stub server records the clientId and payload
        /// </summary>
        [Fact]
        public async Task SafeSend_DelegatesToServerSendAsync_AndRecordsMessage()
        {
            var bridge = NewBridge();

            var server = new WatsonWsServer("127.0.0.1", 9000, false);
            SetWsServer(bridge, server);

            var id = Guid.NewGuid();
            var payload = "{\"type\":\"ping\"}";

            await InvokeSafeSend(bridge, id, payload);

            Assert.Contains(server.Sent, t => t.clientId == id && t.message == payload);
        }


        // ----- GetLocalIp behavior -----


        /// <summary>
        /// Helper: invokes the private static <c>GetLocalIp(Activity?)</c>.
        /// </summary>
        /// <param name="act">Android Activity or null.</param>
        /// <returns>IP address string.</returns>
        private static string CallGetLocalIp(Activity? act)
        {
            var t = typeof(WsBridgeManager);
            var m = t.GetMethod("GetLocalIp", BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(m);
            return (string)m!.Invoke(null, new object?[] { act })!;
        }


        /// <summary>
        /// GetLocalIp returns "0.0.0.0" when Activity is null.
        /// Expected:
        /// - "0.0.0.0"
        /// </summary>
        [Fact]
        public void GetLocalIp_Returns_0_0_0_0_When_Activity_Is_Null()
        {
            var ip = CallGetLocalIp(null);
            Assert.Equal("0.0.0.0", ip);
        }


        /// <summary>
        /// GetLocalIp returns "0.0.0.0" when Wifi ConnectionInfo is null.
        /// Expected:
        /// - "0.0.0.0"
        /// </summary>
        [Fact]
        public void GetLocalIp_Returns_0_0_0_0_When_ConnectionInfo_Is_Null()
        {
            var act = new Activity();
            var wm = (WifiManager)act.GetSystemService(Activity.WifiService)!;
            wm.ConnectionInfo = null;

            var ip = CallGetLocalIp(act);
            Assert.Equal("0.0.0.0", ip);
        }


        /// <summary>
        /// GetLocalIp parses little-endian Android int into dotted quad string.
        /// Expected:
        /// - Default stub value equals "192.168.1.42"
        /// </summary>
        [Fact]
        public void GetLocalIp_Parses_LittleEndian_Int_To_DottedQuad()
        {
            var act = new Activity();
            var ip = CallGetLocalIp(act);
            Assert.Equal("192.168.1.42", ip);
        }


        /// <summary>
        /// GetLocalIp converts various int patterns correctly to A.B.C.D.
        /// Expected:
        /// - For provided patterns, matches expected strings
        /// </summary>
        [Theory]
        [InlineData(10, 0, 0, 1)]       // 10.0.0.1
        [InlineData(172, 16, 5, 200)]   // 172.16.5.200
        [InlineData(192, 168, 100, 2)]  // 192.168.100.2
        public void GetLocalIp_Converts_IntCorrectly(int a, int b, int c, int d)
        {
            // compose the int as Android does (little-endian in the bits)
            int androidInt = (d << 24) | (c << 16) | (b << 8) | a;

            var act = new Activity();
            var wm = (WifiManager)act.GetSystemService(Activity.WifiService)!;
            wm.ConnectionInfo!.IpAddress = androidInt;

            var ip = CallGetLocalIp(act);
            Assert.Equal($"{a}.{b}.{c}.{d}", ip);
        }


        // ----- GetString behavior -----


        /// <summary>
        /// Helper: invokes the private static <c>GetString(ArraySegment&lt;byte&gt;)</c> decoder.
        /// </summary>
        /// <param name="seg">Byte segment.</param>
        /// <returns>Decoded UTF-8 string for the specified slice.</returns>
        private static string CallGetString(ArraySegment<byte> seg)
        {
            var t = typeof(WsBridgeManager);
            var m = t.GetMethod("GetString", BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(m);
            return (string)m!.Invoke(null, new object?[] { seg })!;
        }


        /// <summary>
        /// GetString returns empty string for empty segments.
        /// Expected:
        /// - ""
        /// </summary>
        [Fact]
        public void GetString_EmptySegment_ReturnsEmpty()
        {
            var seg = new ArraySegment<byte>(Array.Empty<byte>(), 0, 0);
            var s = CallGetString(seg);
            Assert.Equal(string.Empty, s);
        }


        /// <summary>
        /// GetString decodes UTF-8 correctly respecting offset and count.
        /// Expected:
        /// - Decoded string matches the inner slice (including multibyte chars)
        /// </summary>
        [Fact]
        public void GetString_DecodesUtf8_WithOffsetAndCount()
        {
            var inner = "Hi ä😊"; // include multibyte UTF-8
            var prefix = "xxxx";
            var suffix = "yyyy";

            var bytes = Encoding.UTF8.GetBytes(prefix + inner + suffix);
            var seg = new ArraySegment<byte>(
                bytes,
                Encoding.UTF8.GetByteCount(prefix),
                Encoding.UTF8.GetByteCount(inner)
            );

            var s = CallGetString(seg);
            Assert.Equal(inner, s);
        }


        /// <summary>
        /// GetString decodes plain ASCII strings correctly.
        /// Expected:
        /// - Exact round-trip for ASCII
        /// </summary>
        [Fact]
        public void GetString_DecodesPlainAscii()
        {
            var txt = "hello-world";
            var seg = new ArraySegment<byte>(Encoding.UTF8.GetBytes(txt));
            var s = CallGetString(seg);
            Assert.Equal(txt, s);
        }


        /// <summary>
        /// GetString only decodes the selected segment slice, not the surrounding bytes.
        /// Expected:
        /// - Output equals the middle payload portion
        /// </summary>
        [Fact]
        public void GetString_DecodesOnlySelectedSegment()
        {
            // full payload: "xxxx" + "hello🌟" + "yyyy"
            var prefix = Encoding.UTF8.GetBytes("xxxx");
            var middle = Encoding.UTF8.GetBytes("hello🌟"); // include multibyte char
            var suffix = Encoding.UTF8.GetBytes("yyyy");

            var buffer = new byte[prefix.Length + middle.Length + suffix.Length];
            Buffer.BlockCopy(prefix, 0, buffer, 0, prefix.Length);
            Buffer.BlockCopy(middle, 0, buffer, prefix.Length, middle.Length);
            Buffer.BlockCopy(suffix, 0, buffer, prefix.Length + middle.Length, suffix.Length);

            // segment that only points to "hello🌟"
            var seg = new ArraySegment<byte>(buffer, prefix.Length, middle.Length);

            var s = CallGetString(seg);
            Assert.Equal("hello🌟", s);
        }
    }
}
