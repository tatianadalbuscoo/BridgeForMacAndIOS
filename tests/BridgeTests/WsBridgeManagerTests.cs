using System;
using System.Linq;
using Xunit;
using ShimmerBridgeMangager;
using System.Text.Json;
using System.Threading.Tasks;
using ShimmerAPI;
using WatsonWebsocket;
using System.Reflection;
using Android.Net.Wifi;
using System.Text;




namespace tests.BridgeTests
{
    public class ExgModeTests
    {
        // -------- Enum shape & stability --------

        /// <summary>
        /// Verifies the underlying integer values are stable and ordered as defined.
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
        /// </summary>
        [Fact]
        public void Names_Match_Expected_Set()
        {
            var names = Enum.GetNames(typeof(ExgMode));
            Assert.Equal(new[] { "None", "ECG", "EMG", "ExgTest", "Respiration" }, names);
        }

        /// <summary>
        /// Verifies the enum has no [Flags] attribute (modes are mutually exclusive).
        /// </summary>
        [Fact]
        public void HasNoFlagsAttribute()
        {
            var hasFlags = Attribute.IsDefined(typeof(ExgMode), typeof(FlagsAttribute));
            Assert.False(hasFlags);
        }

        // -------- String conversion & parsing --------

        /// <summary>
        /// Verifies ToString returns the symbolic name (not the numeric value).
        /// </summary>
        [Fact]
        public void ToString_Returns_Name()
        {
            Assert.Equal("ECG", ExgMode.ECG.ToString());
            Assert.Equal("ExgTest", ExgMode.ExgTest.ToString());
        }

        /// <summary>
        /// Verifies Enum.TryParse works in a case-insensitive way for valid names.
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
        /// Verifies parsing fails for unknown names and does not mutate the out var.
        /// </summary>
        [Fact]
        public void TryParse_Fails_OnUnknown()
        {
            var ok = Enum.TryParse<ExgMode>("resp", ignoreCase: false, out var mode);
            // "resp" (lowercase) is not an enum member name; case-sensitive parse should fail.
            Assert.False(ok);
            // Out var defaults to zero when TryParse fails (default(ExgMode) == None).
            Assert.Equal(ExgMode.None, mode);
        }

        // -------- Enumeration helpers --------

        /// <summary>
        /// Verifies Enum.GetValues returns all declared items without duplicates.
        /// </summary>
        [Fact]
        public void GetValues_Covers_AllItems()
        {
            var vals = Enum.GetValues(typeof(ExgMode)).Cast<ExgMode>().ToArray();
            Assert.Equal(5, vals.Length);
            Assert.Equal(new[] { ExgMode.None, ExgMode.ECG, ExgMode.EMG, ExgMode.ExgTest, ExgMode.Respiration }, vals);
        }

        // -------- Defaults --------

        /// <summary>
        /// Verifies default values of <see cref="ShimmerConfig"/>.
        /// Expected:
        /// - All IMU flags: false
        /// - SamplingRate == null
        /// - EXG flags: EnableExg1/2 false, ExgUse16Bit false
        /// - ExgMode == None
        /// - ExgModeWire == null (omitted in JSON)
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

        // -------- Setters --------

        /// <summary>
        /// Verifies that setters persist values for all IMU/EXG flags and SamplingRate.
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

        // -------- exg_mode wire mapping --------

        /// <summary>
        /// Verifies round-trip mapping between <see cref="ShimmerConfig.ExgMode"/> and the JSON wire field "exg_mode".
        /// Expected:
        /// - Enum -> wire string on get
        /// - Wire string (case/format variants) -> Enum on set
        /// - null/blank resets to None and omits from JSON
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

            c.ExgModeWire = "ExgTest";   // alias allowed
            Assert.Equal(ExgMode.ExgTest, c.ExgMode);

            c.ExgModeWire = "respiration"; // alias allowed
            Assert.Equal(ExgMode.Respiration, c.ExgMode);

            // Reset via null/blank
            c.ExgModeWire = null;
            Assert.Equal(ExgMode.None, c.ExgMode);
            Assert.Null(c.ExgModeWire);

            var json = JsonSerializer.Serialize(c);
            Assert.DoesNotContain("exg_mode", json);
        }

        /// <summary>
        /// Verifies JSON includes "exg_mode" only when non-null.
        /// </summary>
        [Fact]
        public void Json_Serializes_ExgModeWire_WhenSet()
        {
            var c = new ShimmerConfig { ExgMode = ExgMode.ECG };
            var json = JsonSerializer.Serialize(c);
            Assert.Contains("\"exg_mode\":\"ecg\"", json);
        }

        // -------- AnySensorEnabled helper --------

        /// <summary>
        /// Verifies <see cref="WsBridgeManager.AnySensorEnabled(ShimmerConfig)"/> returns false when all flags are off.
        /// </summary>
        [Fact]
        public void AnySensorEnabled_False_WhenAllDisabled()
        {
            var c = new ShimmerConfig(); // all defaults are false/null
            Assert.False(WsBridgeManager.AnySensorEnabled(c));
        }

        /// <summary>
        /// Verifies <see cref="WsBridgeManager.AnySensorEnabled(ShimmerConfig)"/> returns true if any single flag is on.
        /// Covers one EXG flag and one IMU flag as representatives.
        /// </summary>
        [Fact]
        public void AnySensorEnabled_True_WhenAnyOn()
        {
            // EXG representative
            var c1 = new ShimmerConfig { EnableExg1 = true };
            Assert.True(WsBridgeManager.AnySensorEnabled(c1));

            // IMU representative
            var c2 = new ShimmerConfig { EnableGyroscope = true };
            Assert.True(WsBridgeManager.AnySensorEnabled(c2));
        }

        // ExgModeWire

        // -------- Getter mapping --------

        /// <summary>
        /// Getter returns null when ExgMode == None (so JSON può omettere il campo).
        /// </summary>
        [Fact]
        public void Getter_ReturnsNull_WhenModeNone()
        {
            var c = new ShimmerConfig { ExgMode = ExgMode.None };
            Assert.Null(c.ExgModeWire);
        }

        /// <summary>
        /// Getter maps enum values to the expected lowercase wire strings.
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

        // -------- Setter parsing --------

        /// <summary>
        /// Setter accetta varianti case-insensitive/sinonimi e imposta l'enum corretto.
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

            c.ExgModeWire = "ExgTest";   // sinonimo
            Assert.Equal(ExgMode.ExgTest, c.ExgMode);

            c.ExgModeWire = "exg_test";  // sinonimo con underscore
            Assert.Equal(ExgMode.ExgTest, c.ExgMode);

            c.ExgModeWire = "resp";
            Assert.Equal(ExgMode.Respiration, c.ExgMode);

            c.ExgModeWire = "respiration"; // sinonimo esteso
            Assert.Equal(ExgMode.Respiration, c.ExgMode);
        }

        /// <summary>
        /// Setter con null/empty/whitespace resetta a ExgMode.None.
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
        /// Setter con valore sconosciuto resetta a ExgMode.None.
        /// </summary>
        [Fact]
        public void Setter_Unknown_ResetsToNone()
        {
            var c = new ShimmerConfig { ExgMode = ExgMode.ECG };

            c.ExgModeWire = "unknown";
            Assert.Equal(ExgMode.None, c.ExgMode);
        }

        // -------- Round-trip --------

        /// <summary>
        /// Round-trip: set via wire string -> enum corretto -> getter restituisce lo stesso wire string canonico.
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

        // -------- JSON behavior --------

        /// <summary>
        /// Con JsonIgnore(WhenWritingNull): quando ExgModeWire è null, la serializzazione NON include "exg_mode".
        /// </summary>
        [Fact]
        public void Json_Omits_Field_WhenNull()
        {
            var c = new ShimmerConfig { ExgMode = ExgMode.None };

            string json = JsonSerializer.Serialize(c);
            Assert.DoesNotContain("\"exg_mode\"", json);
        }

        /// <summary>
        /// Quando ExgMode è impostato, la serializzazione include "exg_mode" con la wire-string attesa.
        /// </summary>
        [Fact]
        public void Json_Includes_Field_WhenSet()
        {
            var c = new ShimmerConfig { ExgMode = ExgMode.EMG };

            string json = JsonSerializer.Serialize(c);
            Assert.Contains("\"exg_mode\":\"emg\"", json);
        }

        // WsBridgeManager class
        // -------- IsRunning --------

        /// <summary>
        /// Before StartAsync, IsRunning must be false.
        /// </summary>
        [Fact]
        public void IsRunning_False_BeforeStart()
        {
            var mgr = new WsBridgeManager();
            Assert.False(mgr.IsRunning);
        }

        /// <summary>
        /// After StartAsync, IsRunning must be true (Watson stub flips IsListening on Start).
        /// </summary>
        [Fact]
        public async Task IsRunning_True_AfterStart()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity(), port: 8787);
            Assert.True(mgr.IsRunning);
        }

        /// <summary>
        /// After StopAsync, IsRunning must be false.
        /// </summary>
        [Fact]
        public async Task IsRunning_False_AfterStop()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            await mgr.StopAsync();
            Assert.False(mgr.IsRunning);
        }

        // -------- ActiveSessionCount --------

        /// <summary>
        /// ActiveSessionCount is 0 with no opened device sessions.
        /// (We don't open HW sessions here to keep tests fast and decoupled.)
        /// </summary>
        [Fact]
        public void ActiveSessionCount_Initially_Zero()
        {
            var mgr = new WsBridgeManager();
            Assert.Equal(0, mgr.ActiveSessionCount);
        }

        // -------- Dispose behavior --------

        /// <summary>
        /// Dispose() should be safe and leave IsRunning == false (it calls StopAsync fire-and-forget).
        /// </summary>
        [Fact]
        public async Task Dispose_StopsManager_Safely()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            Assert.True(mgr.IsRunning);

            mgr.Dispose();

            // Give a tiny tick to allow StopAsync (fire-and-forget) to run in background.
            await Task.Delay(50);
            Assert.False(mgr.IsRunning);
        }

        // -------- AnySensorEnabled --------

        /// <summary>
        /// AnySensorEnabled returns false when all sensor flags are disabled and no EXG is on.
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
        /// AnySensorEnabled returns true as soon as any single IMU flag is enabled.
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
            var c = new ShimmerConfig(); // tutto false/null per default

            // riflessione semplice per attivare 1 flag
            var prop = typeof(ShimmerConfig).GetProperty(flagName);
            Assert.NotNull(prop);
            prop!.SetValue(c, true);

            Assert.True(WsBridgeManager.AnySensorEnabled(c));
        }

        /// <summary>
        /// AnySensorEnabled returns true when EXG1 or EXG2 is enabled.
        /// </summary>
        [Fact]
        public void AnySensorEnabled_True_WhenExgEnabled()
        {
            var c1 = new ShimmerConfig { EnableExg1 = true };
            var c2 = new ShimmerConfig { EnableExg2 = true };

            Assert.True(WsBridgeManager.AnySensorEnabled(c1));
            Assert.True(WsBridgeManager.AnySensorEnabled(c2));
        }

        // -------- Log event (sanity check) --------

        /// <summary>
        /// Start/Stop should emit some log messages via Log event; we assert at least one arrives.
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

        // StartAsync behavior
        /// <summary>
        /// StartAsync should flip IsRunning to true and log the listen URL.
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
        /// Calling StartAsync twice should be a no-op (still running, no exception).
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

        /// <summary>
        /// After StopAsync the server should not be running.
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

        // StopAsync behavior
        /// <summary>
        /// StopAsync should be safe when the server was never started:
        /// no exception, IsRunning stays false, and "WS stopped" is logged.
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
        /// After starting, StopAsync should turn IsRunning to false and log the stop.
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
        /// Multiple calls to StopAsync should be idempotent and not throw.
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

        // ------------------------------------------------------------
        // ----- WS lifecycle behavior (no SPP dependencies) ----------
        // ------------------------------------------------------------

        /// <summary>
        /// Start -> Stop -> Start again should be safe:
        /// - no exceptions
        /// - IsRunning true after each Start
        /// - IsRunning false after Stop
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
        /// Multiple Start/Stop cycles should not leak state:
        /// we only assert the final IsRunning == false and no exceptions are thrown.
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
        /// Log event should fire at least once during a start-stop cycle
        /// (sanity check that instrumentation is wired).
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
        /// Dispose() when not running should be a no-op and must not throw.
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
        /// AnySensorEnabled should not consider SamplingRate alone as "sensors enabled".
        /// (SamplingRate is independent from enabling any IMU/EXG flag.)
        /// </summary>
        [Fact]
        public void AnySensorEnabled_SamplingRate_Alone_IsFalse()
        {
            var c = new ShimmerConfig { SamplingRate = 256.0 };
            Assert.False(WsBridgeManager.AnySensorEnabled(c));
        }

        /// <summary>
        /// JSON round-trip keeps the exg_mode mapping:
        /// - When ExgMode is set, it serializes to "exg_mode":"<wire>"
        /// - Deserializing restores the same enum.
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

        /// <summary>
        /// StartAsync logs a URL; we assert it looks like a ws:// URL and contains the requested port.
        /// (We don't depend on the exact IP to avoid fragility.)
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

        // UpdateConfigAsync behavior
        // ------------------------------------------------------------
        // ----- UpdateConfigAsync behavior (no source changes) -------
        // ------------------------------------------------------------
        [Fact]
        public async Task UpdateConfigAsync_Ignores_When_SessionMissing_And_Logs()
        {
            var mgr = new WsBridgeManager();
            var logs = new System.Collections.Generic.List<string>();
            mgr.Log += s => logs.Add(s);

            await mgr.StartAsync(new Activity());

            await mgr.UpdateConfigAsync("11:22:33:44:55:66", new ShimmerConfig { EnableGyroscope = true });

            // Assert PRIMA di StopAsync (oppure mantieni la lista e verifica dopo)
            Assert.Contains(logs, l => l.Contains("UpdateConfig ignored: session not found"));

            await mgr.StopAsync();
        }


        [Fact]
        public async Task UpdateConfigAsync_Preserves_ExgMode_And_Applies_NewFlags_AndLogs()
        {
            var mgr = new WsBridgeManager();

            // collezioniamo i log per asserzioni puntuali
            var logs = new System.Collections.Generic.List<string>();
            mgr.Log += s => logs.Add(s);

            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:CC";

            // 1) Apriamo la sessione con un ExgMode iniziale (ECG) e un blocco IMU per far partire lo stream
            var initial = new ShimmerConfig
            {
                ExgMode = ExgMode.ECG,
                EnableGyroscope = true,     // almeno un sensore per far "Start()"
                SamplingRate = 51
            };
            await mgr.OpenConfigureAndStartAsync(mac, initial);

            // sanity: dovremmo aver loggato l'exg mode richiesto e applicato
            Assert.Contains(logs, l => l.Contains("[SERVER] applied   exg_mode") && l.Contains("'ecg'"));

            // 2) Aggiorniamo la config provando a cambiare ExgMode (EMG) + nuovi flag + nuova SR
            var update = new ShimmerConfig
            {
                ExgMode = ExgMode.EMG,        // NON deve passare: il server lo blocca al valore corrente (ECG)
                EnableGyroscope = false,      // spegni gyro...
                EnableMagnetometer = true,    // ...accendi mag
                SamplingRate = 200            // nuova SR
            };

            await mgr.UpdateConfigAsync(mac, update);

            // 3) Asserzioni sui log:
            //    a) Il log "update requested" mostra la richiesta EMG e che la modalità è lockata
            Assert.Contains(logs, l => l.Contains("[SERVER] update requested exg_mode") &&
                                       l.Contains("emg") &&
                                       l.Contains("mode lock=True"));

            //    b) Il log "update applied" deve mostrare ancora 'ecg' (modalità preservata)
            Assert.Contains(logs, l => l.Contains("[SERVER] update applied  exg_mode") &&
                                       l.Contains("'ecg'") &&
                                       l.Contains("enum=ECG"));

            //    c) I log di ApplyConfigAsync riportano la SR aggiornata a 200 Hz
            Assert.Contains(logs, l => l.Contains("[CFG] applied (SR=200Hz"));

            //    d) Log finale di reconfig andato a buon fine
            Assert.Contains(logs, l => l.Contains($"[SERVER] Reconfigured {mac} (mode locked)"));

            await mgr.StopAsync();
        }

        // CloseAllAsync behavior
        /// <summary>
        /// CloseAllAsync senza sessioni: non deve lanciare e deve loggare il messaggio atteso.
        /// </summary>
        [Fact]
        public async Task CloseAllAsync_NoSessions_IsSafe_And_Logs()
        {
            var mgr = new WsBridgeManager();
            var logs = new System.Collections.Generic.List<string>();
            mgr.Log += s => logs.Add(s);

            // Non serve avviare il WS per questo test
            Assert.Equal(0, mgr.ActiveSessionCount);

            var ex = await Record.ExceptionAsync(async () => await mgr.CloseAllAsync());
            Assert.Null(ex);

            Assert.Contains(logs, l => l.Contains("[SERVER] All sessions closed"));
            Assert.Equal(0, mgr.ActiveSessionCount);
        }

        /// <summary>
        /// Con più sessioni aperte: CloseAllAsync deve chiudere (Dispose) le sessioni,
        /// svuotare la mappa e lasciare il WS server intatto (se in esecuzione).
        /// </summary>
        [Fact]
        public async Task CloseAllAsync_WithSessions_ClosesAndClears_WithoutStoppingServer()
        {
            var mgr = new WsBridgeManager();
            var logs = new System.Collections.Generic.List<string>();
            mgr.Log += s => logs.Add(s);

            // Avvio WS per verificare che CloseAllAsync non lo tocchi
            await mgr.StartAsync(new Activity());
            Assert.True(mgr.IsRunning);

            // Apriamo due sessioni HW (gli stub di Connect sollevano lo stato CONNECTED)
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

            // Act
            await mgr.CloseAllAsync();

            // Assert: tutte le sessioni chiuse e conteggio azzerato
            Assert.Equal(0, mgr.ActiveSessionCount);
            Assert.Contains(logs, l => l.Contains("[SERVER] All sessions closed"));

            // Il server WS resta in ascolto (CloseAllAsync non lo tocca)
            Assert.True(mgr.IsRunning);

            // Cleanup
            await mgr.StopAsync();
            Assert.False(mgr.IsRunning);
        }

        /// <summary>
        /// CloseAllAsync è idempotente: chiamarlo più volte non deve lanciare né cambiare lo stato finale.
        /// </summary>
        [Fact]
        public async Task CloseAllAsync_IsIdempotent()
        {
            var mgr = new WsBridgeManager();

            // Apri una sessione per avere stato "non vuoto"
            var mac = "00:06:66:AA:BB:CC";
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig
            {
                ExgMode = ExgMode.ECG,
                EnableGyroscope = true,
                SamplingRate = 51
            });
            Assert.True(mgr.ActiveSessionCount >= 1);

            // Prima chiusura
            await mgr.CloseAllAsync();
            Assert.Equal(0, mgr.ActiveSessionCount);

            // Seconda chiusura (nessuna eccezione)
            var ex = await Record.ExceptionAsync(async () => await mgr.CloseAllAsync());
            Assert.Null(ex);
            Assert.Equal(0, mgr.ActiveSessionCount);
        }

        // OnMessage behavior
        [Fact]
        public async Task OnMessage_TextFrame_IsHandled_And_LogsInbound()
        {
            var mgr = new WsBridgeManager();
            var logs = new System.Collections.Generic.List<string>();
            mgr.Log += s => logs.Add(s);

            await mgr.StartAsync(new Activity());

            // prendi il server WS privato via reflection
            var wsField = typeof(WsBridgeManager).GetField("_ws", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var ws = (WatsonWebsocket.WatsonWsServer)wsField!.GetValue(mgr)!;

            var clientId = Guid.NewGuid();
            ws.RaiseConnected(clientId);

            // manda un frame testuale valido ("hello")
            ws.RaiseText(clientId, "{\"type\":\"hello\"}");

            // dai tempo all'async-void handler
            await Task.Delay(50);

            // HandleTextAsync logga "WS IN [...] type=hello ..."
            Assert.Contains(logs, l => l.Contains("WS IN [") && l.Contains("type=hello"));

            await mgr.StopAsync();
        }

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

            // invia un frame Binary (usa il nuovo helper RaiseBinary)
            ws.RaiseBinary(clientId, new byte[] { 0x01, 0x02, 0x03 });

            await Task.Delay(50);

            // nessun "WS IN [...]" perché i non-Text vengono ignorati
            Assert.DoesNotContain(logs, l => l.Contains("WS IN ["));

            await mgr.StopAsync();
        }

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

            // JSON malformato -> eccezione in HandleTextAsync -> catturata da OnMessage
            ws.RaiseText(clientId, "{ not-json");

            await Task.Delay(50);

            Assert.Contains(logs, l => l.StartsWith("OnMessage error: ", StringComparison.Ordinal));

            await mgr.StopAsync();
        }

        // SendConfigSnapshot behavior

        [Fact]
        public async Task SendConfigSnapshot_NoActiveSession_CompletesWithoutSend()
        {
            var mgr = new WsBridgeManager();

            // Avvia il WS senza alcuna sessione attiva
            await mgr.StartAsync(new Activity());

            // Prendi il server WS via reflection
            var wsField = typeof(WsBridgeManager).GetField("_ws", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var ws = (WatsonWebsocket.WatsonWsServer)wsField!.GetValue(mgr)!;

            ws.Sent.Clear();

            var clientId = Guid.NewGuid();
            var mac = "00:06:66:AA:BB:CC"; // non esiste sessione per questo MAC

            // Invoca il metodo privato SendConfigSnapshot(Guid, string)
            var mi = typeof(WsBridgeManager).GetMethod("SendConfigSnapshot", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            Assert.NotNull(mi);

            var task = (Task)mi!.Invoke(mgr, new object[] { clientId, mac })!;
            await task;

            // Nessun invio eseguito
            Assert.Empty(ws.Sent);

            await mgr.StopAsync();
        }

        [Fact]
        public async Task SendConfigSnapshot_WithActiveSession_Sends_ConfigChanged_ToThatClient()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:01";

            // Crea una sessione attiva (apre, applica config, e avvia se necessario)
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

            // Invoca SendConfigSnapshot
            var mi = typeof(WsBridgeManager).GetMethod("SendConfigSnapshot", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            var task = (Task)mi!.Invoke(mgr, new object[] { clientId, mac })!;
            await task;

            // Verifica un solo invio verso il client corretto
            Assert.Single(ws.Sent);
            Assert.Equal(clientId, ws.Sent[0].clientId);

            // Verifica struttura JSON minima
            var payload = ws.Sent[0].message;
            using var doc = System.Text.Json.JsonDocument.Parse(payload);
            var root = doc.RootElement;

            Assert.Equal("config_changed", root.GetProperty("type").GetString());
            Assert.Equal(mac, root.GetProperty("mac").GetString());
            Assert.True(root.TryGetProperty("cfg", out _));
            Assert.True(root.TryGetProperty("available", out var available));
            Assert.Equal(System.Text.Json.JsonValueKind.Array, available.ValueKind);

            // exg_mode serializzato e presenza del blocco gyro tra gli "available"
            var cfg = root.GetProperty("cfg");
            Assert.Equal("ecg", cfg.GetProperty("exg_mode").GetString());
            Assert.Contains(available.EnumerateArray(), x => x.GetString() == "gyro");

            await mgr.StopAsync();
        }

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

            // due snapshot a clientA, uno a clientB
            await (Task)mi!.Invoke(mgr, new object[] { clientA, mac })!;
            await (Task)mi!.Invoke(mgr, new object[] { clientA, mac })!;
            await (Task)mi!.Invoke(mgr, new object[] { clientB, mac })!;

            Assert.Equal(3, ws.Sent.Count);
            Assert.Equal(2, ws.Sent.Count(x => x.clientId == clientA));
            Assert.Equal(1, ws.Sent.Count(x => x.clientId == clientB));

            // tutti i payload devono essere config_changed per il MAC atteso
            foreach (var (_, msg) in ws.Sent)
            {
                using var d = System.Text.Json.JsonDocument.Parse(msg);
                var r = d.RootElement;
                Assert.Equal("config_changed", r.GetProperty("type").GetString());
                Assert.Equal(mac, r.GetProperty("mac").GetString());
            }

            await mgr.StopAsync();
        }

        // HandleTextAsync behavior

        private static WatsonWsServer GetWs(WsBridgeManager mgr)
        {
            var wsField = typeof(WsBridgeManager).GetField("_ws", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            return (WatsonWsServer)wsField!.GetValue(mgr)!;
        }

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

        [Fact]
        public async Task HandleText_SetSamplingRate_BadArgs_ReturnsBadArgs()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();
            // sr mancante e mac vuoto
            ws.RaiseText(client, "{\"type\":\"set_sampling_rate\",\"mac\":\"\",\"sr\":\"oops\"}");

            Assert.Single(ws.Sent);
            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;
            Assert.Equal("set_sampling_rate_ack", root.GetProperty("type").GetString());
            Assert.False(root.GetProperty("ok").GetBoolean());
            Assert.Equal("bad_args", root.GetProperty("error").GetString());

            await mgr.StopAsync();
        }

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

        [Fact]
        public async Task HandleText_Start_AlwaysReturnsStartAck()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();
            // nessuna sessione attiva per smac: ci aspettiamo solo lo start_ack
            ws.RaiseText(client, "{\"type\":\"start\",\"mac\":\"00:06:66:00:00:01\"}");

            // Potrebbe anche inviare open_ack se la sessione esistesse; qui non c'è.
            Assert.Single(ws.Sent);
            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;
            Assert.Equal("start_ack", root.GetProperty("type").GetString());
            Assert.True(root.GetProperty("ok").GetBoolean());
            Assert.Equal("server_managed", root.GetProperty("note").GetString());

            await mgr.StopAsync();
        }

        [Fact]
        public async Task HandleText_ListDevices_WithNoBondedDevices_ReturnsEmptyItems()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());
            var ws = GetWs(mgr);
            ws.Sent.Clear();

            // Assicuriamoci che non ci siano paired devices negli stub
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

        // TryGetConfig behavior


        [Fact]
        public async Task GetConfig_WhenSessionMissing_ReturnsNotActive()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();

            // MAC inesistente -> TryGetConfig deve restituire false e la reply deve avere ok=false, error=not_active
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

        [Fact]
        public async Task GetConfig_WhenSessionExists_ReturnsEffectiveConfig()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:CC";

            // Apri una sessione attiva con una config riconoscibile
            var initialCfg = new ShimmerConfig
            {
                EnableGyroscope = true,
                EnableBattery = true,
                SamplingRate = 64,
                ExgMode = ExgMode.EMG   // così ExgModeWire dovrebbe essere "emg"
            };

            // Crea davvero la sessione (gli stub fanno completare subito la Connect)
            await mgr.OpenConfigureAndStartAsync(mac, initialCfg);

            var ws = GetWs(mgr);
            ws.Sent.Clear();

            var client = Guid.NewGuid();

            // Ora TryGetConfig deve trovare la sessione e restituire la config corrente
            ws.RaiseText(client, $"{{\"type\":\"get_config\",\"mac\":\"{mac}\"}}");

            Assert.Single(ws.Sent);
            Assert.Equal(client, ws.Sent[0].clientId);

            using var doc = JsonDocument.Parse(ws.Sent[0].message);
            var root = doc.RootElement;

            Assert.Equal("config", root.GetProperty("type").GetString());
            Assert.True(root.GetProperty("ok").GetBoolean());
            Assert.Equal(mac, root.GetProperty("mac").GetString());

            var cfg = root.GetProperty("cfg");
            // flag che abbiamo impostato
            Assert.True(cfg.GetProperty("EnableGyroscope").GetBoolean());
            Assert.True(cfg.GetProperty("EnableBattery").GetBoolean());

            // sampling rate arrotondato a int dal codice: qui 64
            Assert.Equal(64, cfg.GetProperty("SamplingRate").GetInt32());

            // ExgModeWire serializzato come "emg"
            Assert.Equal("emg", cfg.GetProperty("exg_mode").GetString());

            await mgr.StopAsync();
        }

        // Subscribe behavior

        // Helper riflessivo: legge la HashSet<string> dei MAC per un dato clientId
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

        [Fact]
        public async Task Open_SubscribesClient_ToGivenMac_AndLogs()
        {
            var mgr = new WsBridgeManager();
            var logs = new List<string>();
            mgr.Log += s => logs.Add(s);

            await mgr.StartAsync(new Activity());

            // Serve una sessione attiva per poter entrare nel ramo "open" che chiama Subscribe
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

            // Invia il comando "open"
            ws!.RaiseText(client, $"{{\"type\":\"open\",\"mac\":\"{mac}\"}}");

            // Verifica: la sottoscrizione esiste e contiene il MAC
            var set = GetSubscribedSet(mgr, client);
            Assert.NotNull(set);
            Assert.Contains(mac, set!);

            // Log contiene il messaggio di subscribe
            Assert.Contains(logs, s => s.Contains("subscribed", StringComparison.OrdinalIgnoreCase)
                                     && s.Contains(mac, StringComparison.OrdinalIgnoreCase));

            await mgr.StopAsync();
        }

        [Fact]
        public async Task Open_IsIdempotent_AndCaseInsensitive()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:CC";
            // Sessione attiva
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig { EnableBattery = true, SamplingRate = 64 });

            var ws = typeof(WsBridgeManager)
                .GetField("_ws", BindingFlags.NonPublic | BindingFlags.Instance)!
                .GetValue(mgr) as WatsonWebsocket.WatsonWsServer;
            Assert.NotNull(ws);

            var client = Guid.NewGuid();

            // 1ª open (maiuscole)
            ws!.RaiseText(client, $"{{\"type\":\"open\",\"mac\":\"{mac.ToUpperInvariant()}\"}}");
            var set = GetSubscribedSet(mgr, client);
            Assert.NotNull(set);
            Assert.Single(set!);               // solo un elemento
            Assert.Contains(mac, set!);        // confronto HashSet case-insensitive

            // 2ª open (minuscole): deve restare 1 solo elemento
            ws.RaiseText(client, $"{{\"type\":\"open\",\"mac\":\"{mac.ToLowerInvariant()}\"}}");
            set = GetSubscribedSet(mgr, client);
            Assert.NotNull(set);
            Assert.Single(set!);
            Assert.Contains(mac, set!);

            // 3ª open identica: ancora idempotente
            ws.RaiseText(client, $"{{\"type\":\"open\",\"mac\":\"{mac}\"}}");
            set = GetSubscribedSet(mgr, client);
            Assert.NotNull(set);
            Assert.Single(set!);

            await mgr.StopAsync();
        }

        // Unsubscribe behavior


        [Fact]
        public async Task Unsubscribe_RemovesMac_AndLogs()
        {
            var mgr = new WsBridgeManager();
            var logs = new List<string>();
            mgr.Log += s => logs.Add(s);

            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:CC";
            // Serve una sessione attiva perché "open"/"unsubscribe" lavorino su un MAC valido
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

            // 1) subscribe via "open"
            ws!.RaiseText(client, $"{{\"type\":\"open\",\"mac\":\"{mac}\"}}");
            var set = GetSubscribedSet(mgr, client);
            Assert.NotNull(set);
            Assert.Contains(mac, set!);

            // 2) unsubscribe
            ws.RaiseText(client, $"{{\"type\":\"unsubscribe\",\"mac\":\"{mac}\"}}");

            set = GetSubscribedSet(mgr, client);
            // Il set può esistere ancora ma NON deve contenere quel MAC
            if (set != null)
                Assert.DoesNotContain(mac, set);
            // Log contiene il messaggio di unsubscribe
            Assert.Contains(logs, s => s.Contains("unsubscribed", StringComparison.OrdinalIgnoreCase)
                                     && s.Contains(mac, StringComparison.OrdinalIgnoreCase));

            await mgr.StopAsync();
        }

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

            // A) Unsubscribe prima di essere iscritto: non deve esplodere e non deve creare una entry
            ws!.RaiseText(client, $"{{\"type\":\"unsubscribe\",\"mac\":\"{mac}\"}}");
            var set = GetSubscribedSet(mgr, client);
            Assert.Null(set); // nessuna entry creata dal solo unsubscribe

            // B) Iscrizione
            ws.RaiseText(client, $"{{\"type\":\"open\",\"mac\":\"{mac}\"}}");
            set = GetSubscribedSet(mgr, client);
            Assert.NotNull(set);
            Assert.Single(set!);
            Assert.Contains(mac, set!);

            // C) Unsubscribe con casing diverso (case-insensitive)
            ws.RaiseText(client, $"{{\"type\":\"unsubscribe\",\"mac\":\"{mac.ToLowerInvariant()}\"}}");
            set = GetSubscribedSet(mgr, client);
            if (set != null)
                Assert.DoesNotContain(mac, set);

            // D) Unsubscribe ripetuto (idempotente)
            ws.RaiseText(client, $"{{\"type\":\"unsubscribe\",\"mac\":\"{mac}\"}}");
            set = GetSubscribedSet(mgr, client);
            if (set != null)
                Assert.DoesNotContain(mac, set);

            await mgr.StopAsync();
        }

        // BroadcastToSubscribers behavior

        // Helper riflessivo per _ws
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


        [Fact]
        public async Task Broadcast_NoWs_NoThrow()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:11:22:33";
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig { EnableMagnetometer = true, SamplingRate = 51 });

            // forza _ws = null per verificare il guard clause
            var f = typeof(WsBridgeManager).GetField("_ws", BindingFlags.NonPublic | BindingFlags.Instance);
            Assert.NotNull(f);
            f!.SetValue(mgr, null);

            // Non deve lanciare anche se WS è null (BroadcastToSubscribers ha il guard)
            var ex = await Record.ExceptionAsync(async () =>
                await mgr.UpdateConfigAsync(mac, new ShimmerConfig { EnableBattery = true, SamplingRate = 64 }));
            Assert.Null(ex);
        }

        [Fact]
        public async Task Broadcast_WhenNoSubscribers_DoesNothing()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:44:55:66";
            WatsonWsServer.ClearSentLog();

            // Apri sessione HW, nessun iscritto
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig { EnableLowNoiseAccelerometer = true, SamplingRate = 51 });
            Assert.Empty(WatsonWsServer.SentLog);

            // UpdateConfigAsync farà un broadcast, ma non essendoci iscritti non deve inviare nulla
            await mgr.UpdateConfigAsync(mac, new ShimmerConfig { EnableGyroscope = true, SamplingRate = 128 });
            Assert.Empty(WatsonWsServer.SentLog);

            await mgr.StopAsync();
        }


        [Fact]
        public async Task Broadcast_WithOneSubscriber_Completes_NoErrors()
        {
            var mgr = new WsBridgeManager();
            string? lastLog = null;
            mgr.Log += s => lastLog = s;

            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:01";

            // Apri una sessione hardware (non importa il contenuto, serve a rendere attivo il MAC)
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig { EnableGyroscope = true, SamplingRate = 51 });

            // Registra due client; iscriviamo solo A
            var ws = GetWs(mgr);
            var clientA = Guid.NewGuid();
            var clientB = Guid.NewGuid();
            ws.RaiseConnected(clientA);
            ws.RaiseConnected(clientB);

            // "open" sottoscrive il clientA al MAC
            ws.RaiseText(clientA, $"{{\"type\":\"open\",\"mac\":\"{mac}\"}}");

            // Sanity: A è iscritto, B no
            Assert.Contains(mac, GetSubscribedMacs(mgr, clientA));
            Assert.DoesNotContain(mac, GetSubscribedMacs(mgr, clientB));

            // Questo causerà un broadcast "config_changed" ai soli iscritti
            var ex = await Record.ExceptionAsync(async () =>
                await mgr.UpdateConfigAsync(mac, new ShimmerConfig { EnableBattery = true, SamplingRate = 100 })
            );

            Assert.Null(ex);
            // Non avendo ganci su SendAsync, ci limitiamo a verificare che non ci siano errori loggati
            Assert.DoesNotContain("Broadcast error", lastLog ?? string.Empty);

            await mgr.StopAsync();
        }

        [Fact]
        public async Task Broadcast_NoSubscribers_Completes_Silently()
        {
            var mgr = new WsBridgeManager();
            string? lastLog = null;
            mgr.Log += s => lastLog = s;

            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:02";

            // Sessione attiva, ma nessun client "open" ⇒ nessuna sottoscrizione
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig { EnableLowNoiseAccelerometer = true, SamplingRate = 51 });

            var ex = await Record.ExceptionAsync(async () =>
                await mgr.UpdateConfigAsync(mac, new ShimmerConfig { EnableGyroscope = true, SamplingRate = 128 })
            );

            Assert.Null(ex);
            Assert.DoesNotContain("Broadcast error", lastLog ?? string.Empty);

            await mgr.StopAsync();
        }

        [Fact]
        public async Task Broadcast_WhenWsIsNull_NoThrow()
        {
            var mgr = new WsBridgeManager();
            await mgr.StartAsync(new Activity());

            var mac = "00:06:66:AA:BB:03";
            await mgr.OpenConfigureAndStartAsync(mac, new ShimmerConfig { EnableMagnetometer = true, SamplingRate = 51 });

            // Forza _ws = null per testare il guard
            var f = typeof(WsBridgeManager).GetField("_ws", BindingFlags.NonPublic | BindingFlags.Instance);
            Assert.NotNull(f);
            f!.SetValue(mgr, null);

            var ex = await Record.ExceptionAsync(async () =>
                await mgr.UpdateConfigAsync(mac, new ShimmerConfig { EnableBattery = true, SamplingRate = 64 })
            );

            Assert.Null(ex);
        }

        // SendJson behavior

        private static MethodInfo GetSendJsonMI()
        {
            var mi = typeof(WsBridgeManager).GetMethod(
                "SendJson",
                BindingFlags.NonPublic | BindingFlags.Instance
            );
            Assert.NotNull(mi);
            return mi!;
        }

        [Fact]
        public async Task SendJson_NoServer_ReturnsCompletedTask_NoThrow()
        {
            var mgr = new WsBridgeManager();
            string? lastLog = null;
            mgr.Log += s => lastLog = s;

            // _ws è null finché non chiami StartAsync
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
            await t!; // CompletedTask
            // Nessun errore loggato
            Assert.True(string.IsNullOrEmpty(lastLog) || !lastLog!.Contains("WS send error", StringComparison.OrdinalIgnoreCase));
        }

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

        private sealed class Cyclic
        {
            public Cyclic? Self { get; set; }
        }

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
            cyc.Self = cyc; // ciclo per far fallire System.Text.Json

            Task? t = null;
            // Invoke non deve propagare eccezioni del metodo (il catch interno le logga)
            var ex = Record.Exception(() =>
            {
                var ret = mi.Invoke(mgr, new object[] { clientId, (object)cyc });
                t = (Task)ret!;
            });

            Assert.Null(ex);
            Assert.NotNull(t);
            await t!; // deve essere CompletedTask nonostante l'errore di serializzazione

            Assert.NotNull(lastLog);
            Assert.Contains("WS send error", lastLog!, StringComparison.OrdinalIgnoreCase);

            await mgr.StopAsync();
        }

        // SppSession class

        private static Type GetSppSessionType()
        {
            var t = typeof(WsBridgeManager).GetNestedType("SppSession", BindingFlags.NonPublic);
            Assert.NotNull(t);
            return t!;
        }

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

        private static bool GetIsModeLocked(object spp)
        {
            var p = spp.GetType().GetProperty("IsModeLocked",
                BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(p);
            var val = p!.GetValue(spp);
            Assert.IsType<bool>(val);
            return (bool)val!;
        }

        private static void CallLockMode(object spp)
        {
            var m = spp.GetType().GetMethod("LockMode",
                BindingFlags.Public | BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(m);
            m!.Invoke(spp, Array.Empty<object>());
        }

        [Fact]
        public void IsModeLocked_Default_IsFalse()
        {
            var spp = CreateSppSession("00:11:22:33:44:55");
            Assert.False(GetIsModeLocked(spp));
        }

        [Fact]
        public void LockMode_Sets_IsModeLocked_True()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:FF");

            // prima del lock
            Assert.False(GetIsModeLocked(spp));

            // lock
            CallLockMode(spp);

            // dopo il lock
            Assert.True(GetIsModeLocked(spp));
        }

        [Fact]
        public void LockMode_IsIdempotent_RemainsTrue_OnMultipleCalls()
        {
            var spp = CreateSppSession("11:22:33:44:55:66");

            CallLockMode(spp);
            Assert.True(GetIsModeLocked(spp));

            // richiamo multiplo non deve cambiare nulla (rimane true)
            CallLockMode(spp);
            CallLockMode(spp);

            Assert.True(GetIsModeLocked(spp));
        }

        [Fact]
        public void LockMode_DoesNotDepend_OnDelegates()
        {
            // Verifica anche con delegati “reali” (che catturano dati) che non influenzano il lock
            string? lastMac = null;
            string? lastJson = null;
            string? lastLog = null;

            void Broadcast(string mac, string json) { lastMac = mac; lastJson = json; }
            void Log(string s) { lastLog = s; }

            var spp = CreateSppSession("FE:DC:BA:98:76:54", Broadcast, Log);

            Assert.False(GetIsModeLocked(spp));
            CallLockMode(spp);
            Assert.True(GetIsModeLocked(spp));

            // delegati non devono essere stati invocati dal solo LockMode
            Assert.Null(lastMac);
            Assert.Null(lastJson);
            Assert.Null(lastLog);
        }

        // CurrentConfig behavior

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

        private static void SetPrivateCurrentCfg(object spp, ShimmerConfig cfg)
        {
            var f = spp.GetType().GetField("_currentCfg", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(f);
            f!.SetValue(spp, cfg);
        }

        private static ShimmerConfig GetCurrentConfigSnapshot(object spp)
        {
            var p = spp.GetType().GetProperty("CurrentConfig", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
            Assert.NotNull(p);
            var v = p!.GetValue(spp);
            Assert.IsType<ShimmerConfig>(v);
            return (ShimmerConfig)v!;
        }

        private static ShimmerConfig GetPrivateCurrentCfg(object spp)
        {
            var f = spp.GetType().GetField("_currentCfg", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(f);
            var v = f!.GetValue(spp);
            Assert.IsType<ShimmerConfig>(v);
            return (ShimmerConfig)v!;
        }

        [Fact]
        public void CurrentConfig_Returns_ExactValues_Copy()
        {
            var spp = CreateSppSession("00:11:22:33:44:55");

            // stato interno da “applicato”
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
                SamplingRate = 128.5,
                // EXG
                EnableExg1 = true,
                EnableExg2 = false,
                ExgUse16Bit = true,
                ExgMode = ExgMode.Respiration
            };
            SetPrivateCurrentCfg(spp, applied);

            var snap = GetCurrentConfigSnapshot(spp);

            // stessa semantica valori
            Assert.True(snap.EnableLowNoiseAccelerometer);
            Assert.True(snap.EnableWideRangeAccelerometer);
            Assert.True(snap.EnableGyroscope);
            Assert.True(snap.EnableMagnetometer);
            Assert.True(snap.EnablePressureTemperature);
            Assert.True(snap.EnableBattery);
            Assert.True(snap.EnableExtA6);
            Assert.False(snap.EnableExtA7);
            Assert.True(snap.EnableExtA15);
            Assert.Equal(128.5, snap.SamplingRate);

            Assert.True(snap.EnableExg1);
            Assert.False(snap.EnableExg2);
            Assert.True(snap.ExgUse16Bit);
            Assert.Equal(ExgMode.Respiration, snap.ExgMode);

            // non è la stessa istanza
            var internalRef = GetPrivateCurrentCfg(spp);
            Assert.False(Object.ReferenceEquals(snap, internalRef));
        }

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

            // mutazioni sulla snapshot
            snap1.EnableGyroscope = false;
            snap1.EnableBattery = false;
            snap1.EnableExg1 = false;
            snap1.ExgMode = ExgMode.None;
            snap1.SamplingRate = 200;

            // ricava nuova snapshot dall’oggetto → deve riflettere lo stato interno invariato
            var snap2 = GetCurrentConfigSnapshot(spp);
            Assert.True(snap2.EnableGyroscope);
            Assert.True(snap2.EnableBattery);
            Assert.True(snap2.EnableExg1);
            Assert.Equal(ExgMode.EMG, snap2.ExgMode);
            Assert.Equal(51, snap2.SamplingRate);
        }

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

            // muta A → B non deve cambiare
            a.EnableMagnetometer = false;
            a.SamplingRate = 1;

            Assert.True(b.EnableMagnetometer);
            Assert.Equal(100, b.SamplingRate);
        }

        // EnabledBlocks behavior

        // Wrapper comodo: usa la tua overload esistente CreateSppSession(string)
        private object CreateSppSession() => CreateSppSession("11:22:33:44:55");

        // Helper per invocare il metodo EnabledBlocks() (pubblico) sulla SppSession annidata
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

        [Fact]
        public void EnabledBlocks_Empty_When_AllFlagsOff()
        {
            var spp = CreateSppSession();
            SetPrivateCurrentCfg(spp, new ShimmerConfig());
            var blocks = CallEnabledBlocks(spp);
            Assert.Empty(blocks);
        }

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

        [Fact]
        public void EnabledBlocks_Ext_When_AnyExternalAdcOn()
        {
            var spp = CreateSppSession();

            // A6
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableExtA6 = true });
            Assert.Equal(new[] { "ext" }, CallEnabledBlocks(spp));

            // A7
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableExtA7 = true });
            Assert.Equal(new[] { "ext" }, CallEnabledBlocks(spp));

            // A15
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableExtA15 = true });
            Assert.Equal(new[] { "ext" }, CallEnabledBlocks(spp));

            // Combo -> sempre un solo "ext"
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableExtA6 = true, EnableExtA7 = true, EnableExtA15 = true });
            Assert.Equal(new[] { "ext" }, CallEnabledBlocks(spp));
        }

        [Fact]
        public void EnabledBlocks_TempAdds_Temp_And_Press()
        {
            var spp = CreateSppSession();

            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnablePressureTemperature = true });
            var blocks = CallEnabledBlocks(spp);

            // deve contenere entrambe e nel giusto ordine temp poi press
            Assert.Equal(new[] { "temp", "press" }, blocks);
        }

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

        [Fact]
        public void EnabledBlocks_Composite_AllFlags_OrderIsStable()
        {
            var spp = CreateSppSession();

            var cfg = new ShimmerConfig
            {
                EnableExg1 = true,
                EnableLowNoiseAccelerometer = true,
                EnableWideRangeAccelerometer = true,
                EnableGyroscope = true,
                EnableMagnetometer = true,
                EnablePressureTemperature = true,
                EnableBattery = true,
                EnableExtA6 = true // una qualunque tra A6/A7/A15
            };
            SetPrivateCurrentCfg(spp, cfg);

            var blocks = CallEnabledBlocks(spp);

            // ordine atteso secondo l'implementazione
            var expected = new[] { "exg", "lna", "wra", "gyro", "mag", "temp", "press", "vbatt", "ext" };
            Assert.Equal(expected, blocks);
        }

        [Fact]
        public void EnabledBlocks_NoDuplicates_When_MultipleGroupsEnabled()
        {
            var spp = CreateSppSession();

            var cfg = new ShimmerConfig
            {
                EnableExg1 = true,
                EnableExg2 = true,
                EnableExtA6 = true,
                EnableExtA7 = true,
                EnableExtA15 = true
            };
            SetPrivateCurrentCfg(spp, cfg);

            var blocks = CallEnabledBlocks(spp);

            // exg ed ext compaiono una sola volta
            Assert.Equal(new[] { "exg", "ext" }, blocks);
        }

        [Fact]
        public void EnabledBlocks_OrderStable_RegardlessOfFlagSetOrder()
        {
            var spp = CreateSppSession();

            // Imposta i flag in ordine “strano”
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
            var expected = new[] { "exg", "lna", "wra", "gyro", "mag", "temp", "press", "vbatt", "ext" };
            Assert.Equal(expected, blocks);
        }

        // ResetIndices behavior
        // --- ResetIndices behavior --------------------------------------------------

        private static void SetPrivateIndex(object spp, string field, int value)
        {
            var f = spp.GetType().GetField(field, BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(f);
            f!.SetValue(spp, value);
        }

        private static int GetPrivateIndex(object spp, string field)
        {
            var f = spp.GetType().GetField(field, BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(f);
            return (int)f!.GetValue(spp)!;
        }

        private static void InvokeResetIndices(object spp)
        {
            var m = spp.GetType().GetMethod("ResetIndices", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(m);
            m!.Invoke(spp, null);
        }

        [Fact]
        public void ResetIndices_SetsAllCachedIndicesToMinusOne_And_IsIdempotent()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:FF");

            // Precarica valori != -1 per verificare che vengano azzerati
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

            // Assert: tutti a -1
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

            // Idempotenza: richiamarlo ancora non cambia il risultato
            InvokeResetIndices(spp);
            Assert.Equal(-1, GetPrivateIndex(spp, "iTs"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iExg1"));
            Assert.Equal(-1, GetPrivateIndex(spp, "iA15"));
        }

        // RefreshMissingIndices behavior

        // --- RefreshMissingIndices behavior ----------------------------------------

        private static void InvokeRefreshMissingIndices(object spp, ShimmerAPI.ObjectCluster oc)
        {
            var m = spp.GetType().GetMethod("RefreshMissingIndices", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(m);
            m!.Invoke(spp, new object[] { oc });
        }

        [Fact]
        public void RefreshMissingIndices_DoesNotOverwrite_AlreadyResolvedIndices()
        {
            var spp = CreateSppSession("10:20:30:40:50:60");

            // Abilita EXG1 nel cfg interno
            var cfg = new ShimmerConfig { EnableExg1 = true };
            SetPrivateCurrentCfg(spp, cfg);

            // Preimposta un indice "già risolto"
            var fExg1 = spp.GetType().GetField("iExg1", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fExg1);
            fExg1!.SetValue(spp, 7);

            // Chiama il refresh: gli stub di ObjectCluster restituiscono -1,
            // ma siccome iExg1 != -1, non deve essere toccato.
            var oc = new ShimmerAPI.ObjectCluster();
            InvokeRefreshMissingIndices(spp, oc);

            Assert.Equal(7, (int)fExg1.GetValue(spp)!);
        }

        [Fact]
        public void RefreshMissingIndices_WhenAllFlagsDisabled_KeepsAllAtMinusOne()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:01");

            // Tutti i flag a false -> il metodo non deve cercare indici (restano -1)
            SetPrivateCurrentCfg(spp, new ShimmerConfig());

            // Prepara tutti gli indici a -1
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

        [Fact]
        public void RefreshMissingIndices_TimestampIndex_NotOverwritten_WhenAlreadySet()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:02");

            // Qualunque cfg va bene: qui verifichiamo solo che iTs non venga sovrascritto
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableGyroscope = true });

            // Imposta iTs già risolto
            var fTs = spp.GetType().GetField("iTs", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fTs);
            fTs!.SetValue(spp, 42);

            // Anche se ObjectCluster (stub) restituirà -1 su qualsiasi lookup,
            // iTs non deve cambiare perché diverso da -1
            var oc = new ShimmerAPI.ObjectCluster();
            InvokeRefreshMissingIndices(spp, oc);

            Assert.Equal(42, (int)fTs.GetValue(spp)!);
        }

        [Fact]
        public void RefreshMissingIndices_WhenSomeFlagsEnabled_LeavesPrepopulated_OnOthersUnchanged()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:03");

            // Abilita alcune famiglie di sensori
            var cfg = new ShimmerConfig
            {
                EnableExg1 = true,
                EnableLowNoiseAccelerometer = true,
                EnableMagnetometer = true
            };
            SetPrivateCurrentCfg(spp, cfg);

            // Preimposta indici "già trovati"
            spp.GetType().GetField("iExg1", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, 100);
            spp.GetType().GetField("iLnaX", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, 200);

            // Lasciane altri a -1 (non verranno risolti perché lo stub GetIndex -> -1)
            spp.GetType().GetField("iExg2", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, -1);
            spp.GetType().GetField("iLnaY", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, -1);
            spp.GetType().GetField("iMx", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, -1);

            var oc = new ShimmerAPI.ObjectCluster();
            InvokeRefreshMissingIndices(spp, oc);

            // Quelli preimpostati restano invariati
            Assert.Equal(100, (int)spp.GetType().GetField("iExg1", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(spp)!);
            Assert.Equal(200, (int)spp.GetType().GetField("iLnaX", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(spp)!);

            // Quelli a -1 restano -1 (nessuna risoluzione possibile nello stub)
            Assert.Equal(-1, (int)spp.GetType().GetField("iExg2", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(spp)!);
            Assert.Equal(-1, (int)spp.GetType().GetField("iLnaY", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(spp)!);
            Assert.Equal(-1, (int)spp.GetType().GetField("iMx", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(spp)!);
        }

        // SetSamplingRateAsync behavior

        // --- SetSamplingRateAsync behavior ------------------------------------------

        private static void SetPrivateField(object o, string name, object? value)
        {
            var f = o.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(f);
            f!.SetValue(o, value);
        }

        private static T GetPrivateField<T>(object o, string name)
        {
            var f = o.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(f);
            return (T)f!.GetValue(o)!;
        }

        private static void AssignCore(object spp, string mac = "00:11:22:33:44:55")
        {
            // Core fittizio usato dalla sessione
            var core = new ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2("DEV", mac);
            SetPrivateField(spp, "_core", core);
        }

        [Fact]
        public async Task SetSamplingRateAsync_Throws_When_NotOpen()
        {
            var spp = CreateSppSession("10:10:10:10:10:10"); // _core resta null
            await Assert.ThrowsAsync<InvalidOperationException>(() =>
                (Task<double>)spp.GetType()
                    .GetMethod("SetSamplingRateAsync", BindingFlags.Instance | BindingFlags.Public)!
                    .Invoke(spp, new object[] { 128.0 })!);
        }

        [Fact]
        public async Task SetSamplingRateAsync_UpdatesCfg_ResetsIndices_And_TsBase()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:01");
            AssignCore(spp);

            // Prepara cfg interno + indici sporchi
            SetPrivateCurrentCfg(spp, new ShimmerConfig { SamplingRate = 51, EnableGyroscope = true });
            SetPrivateField(spp, "iTs", 5);
            SetPrivateField(spp, "iExg1", 10);
            SetPrivateField(spp, "iGx", 3);
            // ts base valorizzata → deve tornare null
            SetPrivateField(spp, "_tsBase", (double?)123.45);

            // Chiama API
            var applied = await (Task<double>)spp.GetType()
                .GetMethod("SetSamplingRateAsync", BindingFlags.Instance | BindingFlags.Public)!
                .Invoke(spp, new object[] { 127.6 })!;

            // Rounding a intero
            Assert.Equal(128, applied);

            // CurrentConfig riflette il nuovo SR (usiamo la property pubblica)
            var snap = GetCurrentConfigSnapshot(spp);
            Assert.Equal(128, snap.SamplingRate);

            // Indici resettati
            Assert.Equal(-1, GetPrivateField<int>(spp, "iTs"));
            Assert.Equal(-1, GetPrivateField<int>(spp, "iExg1"));
            Assert.Equal(-1, GetPrivateField<int>(spp, "iGx"));

            // Timestamp base azzerato
            Assert.Null(GetPrivateField<double?>(spp, "_tsBase"));
        }

        [Fact]
        public async Task SetSamplingRateAsync_WhenStreamingAndSensorsEnabled_Restarts()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:02");
            AssignCore(spp);

            // Abilita almeno un sensore così AnySensorEnabled == true
            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableGyroscope = true, SamplingRate = 51 });

            // Simula "streaming attivo" (_handler != null)
            SetPrivateField(spp, "_handler", (EventHandler)((_, __) => { }));

            // Esegue
            var applied = await (Task<double>)spp.GetType()
                .GetMethod("SetSamplingRateAsync", BindingFlags.Instance | BindingFlags.Public)!
                .Invoke(spp, new object[] { 64.2 })!;

            Assert.Equal(64, applied);

            // Atteso: Stop() azzera handler poi Start() lo riporta != null.
            // Verifichiamo che, a fine metodo, l'handler sia presente.
            var handlerAfter = GetPrivateField<EventHandler?>(spp, "_handler");
            Assert.NotNull(handlerAfter);

            // E la config aggiornata
            var snap = GetCurrentConfigSnapshot(spp);
            Assert.Equal(64, snap.SamplingRate);
        }

        [Fact]
        public async Task SetSamplingRateAsync_WhenStreamingAndNoSensors_DoesNotRestart()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:03");
            AssignCore(spp);

            // Nessun sensore abilitato → AnySensorEnabled == false
            SetPrivateCurrentCfg(spp, new ShimmerConfig { SamplingRate = 200 });

            // Simula "streaming attivo" (_handler != null)
            var initialHandler = (EventHandler)((_, __) => { });
            SetPrivateField(spp, "_handler", initialHandler);

            // Esegue
            var applied = await (Task<double>)spp.GetType()
                .GetMethod("SetSamplingRateAsync", BindingFlags.Instance | BindingFlags.Public)!
                .Invoke(spp, new object[] { 100.0 })!;

            Assert.Equal(100, applied);

            // Atteso: NON viene eseguito Start(), quindi non deve essere creato/registrato
            // un nuovo handler; il riferimento resta quello che avevamo prima.
            var handlerAfter = GetPrivateField<EventHandler?>(spp, "_handler");
            Assert.Same(initialHandler, handlerAfter);

            // Config aggiornata
            var snap = GetCurrentConfigSnapshot(spp);
            Assert.Equal(100, snap.SamplingRate);
        }


        [Fact]
        public async Task SetSamplingRateAsync_Rounds_To_Integer_And_Logs()
        {
            var logs = new List<string>();
            // Usa il tuo helper CreateSppSession(...) che accetta la MAC; il logger
            // viene già indirizzato alla action interna. Se hai un overload con logger,
            // sostituiscilo; altrimenti qui verifichiamo solo il rounding.
            var spp = CreateSppSession("AA:BB:CC:DD:EE:04");
            AssignCore(spp);

            SetPrivateCurrentCfg(spp, new ShimmerConfig { EnableExg1 = true, SamplingRate = 10 });

            var applied = await (Task<double>)spp.GetType()
                .GetMethod("SetSamplingRateAsync", BindingFlags.Instance | BindingFlags.Public)!
                .Invoke(spp, new object[] { 200.7 })!;

            Assert.Equal(201, applied);

            var snap = GetCurrentConfigSnapshot(spp);
            Assert.Equal(201, snap.SamplingRate);

            // Se nel tuo setup il logger è catturabile, puoi aggiungere un assert tipo:
            // Assert.Contains(logs, s => s.Contains("[CFG] sampling rate set to 201 Hz"));
            // (lasciato commentato per non creare dipendenze se non collezioni i log)
        }

        // SppSession behavior
        // --- SppSession .ctor --------------------------------------------------------

        [Fact]
        public void SppSession_Ctor_TrimsMac_And_AssignsCallbacks()
        {
            // arrange
            var macIn = "  01:23:45:67:89:AB  ";
            Action<string, string> broadcast = (_, __) => { };
            Action<string> log = _ => { };

            // act
            var spp = CreateSppSession(macIn, broadcast, log);

            // assert: MAC viene trim-mato e salvato
            var macStored = GetPrivateField<string>(spp, "_mac");
            Assert.Equal("01:23:45:67:89:AB", macStored);

            // assert: i callback vengono assegnati
            var bcStored = GetPrivateField<Delegate>(spp, "_broadcast");
            var logStored = GetPrivateField<Delegate>(spp, "_log");
            Assert.Same(broadcast, bcStored);
            Assert.Same(log, logStored);
        }

        [Fact]
        public void SppSession_Ctor_AllowsEmptyOrWhitespaceMac_StoresEmptyString()
        {
            // arrange
            var macIn = "   ";
            Action<string, string> broadcast = (_, __) => { };
            Action<string> log = _ => { };

            // act
            var spp = CreateSppSession(macIn, broadcast, log);

            // assert
            var macStored = GetPrivateField<string>(spp, "_mac");
            Assert.Equal(string.Empty, macStored);
        }

        [Fact]
        public void SppSession_Ctor_InitialState_ModeUnlocked()
        {
            // arrange
            Action<string, string> broadcast = (_, __) => { };
            Action<string> log = _ => { };
            var spp = CreateSppSession("00:11:22:33:44:55", broadcast, log);

            // assert: per default la modalità non è lockata
            var isLockedProp = spp.GetType().GetProperty("IsModeLocked", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(isLockedProp);
            Assert.False((bool)isLockedProp!.GetValue(spp)!);
        }

        // OpenAsync behavior
        // --- OpenAsync behavior ------------------------------------------------------

        private static Delegate? GetEventDelegate(object target, string eventName)
        {
            // Gli eventi field-like espongono un backing field con lo stesso nome
            var f = target.GetType().GetField(eventName, BindingFlags.Instance | BindingFlags.NonPublic);
            return (Delegate?)f?.GetValue(target);
        }

        private static async Task InvokeMethodAsync(object target, string methodName, params object[]? args)
        {
            var m = target.GetType().GetMethod(methodName, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
            Assert.NotNull(m);

            var result = m!.Invoke(target, args ?? Array.Empty<object>());

            if (result is Task t)
            {
                await t.ConfigureAwait(false);
            }
            else
            {
                // Se il metodo non è async/Task, consenti comunque l'invocazione "sincrona".
                // Togli l'eccezione se vuoi supportare anche metodi void.
                // throw new InvalidOperationException($"Method '{methodName}' did not return a Task.");
            }
        }


        [Fact]
        public async Task OpenAsync_Completes_Connects_CoreAssigned_AndUnhooksHandler()
        {
            // arrange
            var mac = "01:23:45:67:89:AB";
            var logs = new List<string>();
            void Broadcast(string _, string __) { }
            void Log(string s) => logs.Add(s);

            var spp = CreateSppSession(mac, Broadcast, Log);

            // precondizioni: _core nullo
            Assert.Null(GetPrivateField<object>(spp, "_core"));

            // act
            await InvokeMethodAsync(spp, "OpenAsync");

            // assert: core creato e connesso
            var core = GetPrivateField<object>(spp, "_core");
            Assert.NotNull(core);

            // Shimmer stub espone proprietà Connected
            var connectedProp = core!.GetType().GetProperty("Connected", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(connectedProp);
            Assert.True((bool)connectedProp!.GetValue(core)!);

            // L'handler on state change è stato sganciato dopo il connect
            var evt = GetEventDelegate(core, "UICallback");
            Assert.Null(evt);

            // Log di connessione emesso
            Assert.Contains(logs, l => l.Contains("[BT] Connected to 01:23:45:67:89:AB"));
        }

        [Fact]
        public async Task OpenAsync_TrimsMac_And_LogsUsingTrimmedValue()
        {
            // arrange
            var logs = new List<string>();
            void Broadcast(string _, string __) { }
            void Log(string s) => logs.Add(s);

            var spp = CreateSppSession("  AA:BB:CC:DD:EE:FF  ", Broadcast, Log);

            // act
            await InvokeMethodAsync(spp, "OpenAsync");

            // assert: log usa il MAC trim-mato
            Assert.Contains(logs, l => l.Contains("[BT] Connected to AA:BB:CC:DD:EE:FF"));
        }

        // ApplyConfigAsync behavior

        // --- ApplyConfigAsync behavior ------------------------------------------------

        [Fact]
        public async Task ApplyConfigAsync_Preserves_ExgMode_When_Locked()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:11");

            // Stato interno precedente con ExgMode = EMG
            SetPrivateCurrentCfg(spp, new ShimmerConfig { ExgMode = ExgMode.EMG });

            // Apri la sessione (inizializza _core)
            await InvokeMethodAsync(spp, "OpenAsync");

            // Lock della modalità: il cfg in arrivo NON deve poter cambiare l'ExgMode
            var lockMode = spp.GetType().GetMethod("LockMode", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(lockMode);
            lockMode!.Invoke(spp, null);

            var cfgReq = new ShimmerConfig
            {
                // Proviamo a cambiarla: deve essere ignorata
                ExgMode = ExgMode.Respiration,
                // metti una SR valida per eseguire il percorso "applica SR"
                SamplingRate = 128,
                // accendi qualche flag per avere una bitmap non nulla
                EnableExg1 = true
            };

            await InvokeMethodAsync(spp, "ApplyConfigAsync", cfgReq);

            var snap = GetCurrentConfigSnapshot(spp);
            Assert.Equal(ExgMode.EMG, snap.ExgMode);            // preservato
            Assert.True(snap.EnableExg1);                       // il resto del cfg applicato
            Assert.Equal(128, snap.SamplingRate);
        }

        [Fact]
        public async Task ApplyConfigAsync_Defaults_SamplingRate_To_51_When_MissingOrInvalid()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:12");
            await InvokeMethodAsync(spp, "OpenAsync");

            // SR mancante
            var cfg1 = new ShimmerConfig
            {
                SamplingRate = null,
                EnableGyroscope = true
            };
            await InvokeMethodAsync(spp, "ApplyConfigAsync", cfg1);
            var s1 = GetCurrentConfigSnapshot(spp);
            Assert.Equal(51, s1.SamplingRate);

            // SR non valida (<= 0)
            var cfg2 = new ShimmerConfig
            {
                SamplingRate = 0,
                EnableMagnetometer = true
            };
            await InvokeMethodAsync(spp, "ApplyConfigAsync", cfg2);
            var s2 = GetCurrentConfigSnapshot(spp);
            Assert.Equal(51, s2.SamplingRate);
        }

        [Fact]
        public async Task ApplyConfigAsync_Rounds_SamplingRate_And_Resets_Indices()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:13");
            await InvokeMethodAsync(spp, "OpenAsync");

            // Metti un indice già "risolto" che deve essere resettato da ApplyConfigAsync
            var fTs = spp.GetType().GetField("iTs", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fTs);
            fTs!.SetValue(spp, 777);

            var cfg = new ShimmerConfig
            {
                SamplingRate = 128.4,      // verrà arrotondato a 128
                EnableBattery = true
            };

            await InvokeMethodAsync(spp, "ApplyConfigAsync", cfg);

            var snap = GetCurrentConfigSnapshot(spp);
            Assert.Equal(128, snap.SamplingRate);

            // Gli indici devono essere resettati a -1
            Assert.Equal(-1, (int)fTs.GetValue(spp)!);
        }

        [Fact]
        public async Task ApplyConfigAsync_Applies_Flags_And_EnabledBlocks_Match_Config_When_Detection_Fails()
        {
            // Nota: il nostro stub TryDetectBoardKind() ritorna false → "using requested flags"
            var spp = CreateSppSession("AA:BB:CC:DD:EE:14");
            await InvokeMethodAsync(spp, "OpenAsync");

            var cfg = new ShimmerConfig
            {
                SamplingRate = 200,
                EnableExg1 = true,
                EnableLowNoiseAccelerometer = true,
                EnableGyroscope = true,
                EnableBattery = true,
                EnableExtA6 = true
            };

            await InvokeMethodAsync(spp, "ApplyConfigAsync", cfg);

            var snap = GetCurrentConfigSnapshot(spp);

            // Verifica lo stato configurazione
            Assert.Equal(200, snap.SamplingRate);
            Assert.True(snap.EnableExg1);
            Assert.True(snap.EnableLowNoiseAccelerometer);
            Assert.True(snap.EnableGyroscope);
            Assert.True(snap.EnableBattery);
            Assert.True(snap.EnableExtA6);

            // Verifica la lista simbolica dei blocchi abilitati (ordine definito dall’implementazione)
            var blocks = CallEnabledBlocks(spp);
            Assert.Equal(new[] { "exg", "lna", "gyro", "vbatt", "ext" }, blocks);
        }

        [Fact]
        public async Task ApplyConfigAsync_ImuOnly_Request_Remains_Imu_When_Detection_Fails()
        {
            // Con detection che fallisce, non forziamo EXG off/on: usiamo i flag richiesti.
            var spp = CreateSppSession("AA:BB:CC:DD:EE:15");
            await InvokeMethodAsync(spp, "OpenAsync");

            var cfg = new ShimmerConfig
            {
                SamplingRate = 100,
                // IMU flags
                EnableLowNoiseAccelerometer = true,
                EnableWideRangeAccelerometer = true,
                EnableGyroscope = true,
                EnableMagnetometer = true,
                EnablePressureTemperature = true,
                EnableBattery = true,
                // EXG spenti
                EnableExg1 = false,
                EnableExg2 = false
            };

            await InvokeMethodAsync(spp, "ApplyConfigAsync", cfg);

            var snap = GetCurrentConfigSnapshot(spp);
            Assert.Equal(100, snap.SamplingRate);
            Assert.False(snap.EnableExg1);
            Assert.False(snap.EnableExg2);

            var blocks = CallEnabledBlocks(spp);
            // Ordine: exg, lna, wra, gyro, mag, temp, press, vbatt, ext
            Assert.Equal(new[] { "lna", "wra", "gyro", "mag", "temp", "press", "vbatt" }, blocks);
        }

        // Start behavior
        // --- Start() behavior --------------------------------------------------------

        [Fact]
        public void Start_Throws_When_NotOpen()
        {
            var spp = CreateSppSession("01:23:45:67:89:AB");
            var mStart = spp.GetType().GetMethod("Start", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mStart);

            var ex = Assert.Throws<TargetInvocationException>(() => mStart!.Invoke(spp, null));
            Assert.IsType<InvalidOperationException>(ex.InnerException);
        }



        [Fact]
        public void Start_Resets_Indices_And_TsBase()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:01");

            // Inietto _core così Start non lancia
            var core = new ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2("Stub", "AA:BB:CC:DD:EE:01");
            var fCore = spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fCore);
            fCore!.SetValue(spp, core);

            // Metto indici e ts base “sporchi”
            spp.GetType().GetField("iTs", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, 99);
            spp.GetType().GetField("_tsBase", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, 123.45);

            // Chiama Start()
            spp.GetType().GetMethod("Start", BindingFlags.Instance | BindingFlags.Public)!.Invoke(spp, null);

            Assert.Equal(-1, (int)spp.GetType().GetField("iTs", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(spp)!);
            Assert.Null(spp.GetType().GetField("_tsBase", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(spp));
        }

        [Fact]
        public void Start_Wires_Handler_And_Broadcasts_Sample_On_DataPacket()
        {
            var mac = "AA:BB:CC:DD:EE:02";
            var sent = new List<(string Mac, string Json)>();
            var spp = CreateSppSession(mac, (m, j) => sent.Add((m, j)));

            // Inietto _core
            var core = new ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2("Stub", mac);
            spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, core);

            // Config minima (tutti i flag off; Exg1/2 non abilitati ma payload con Exg1/Exg2 sempre presente a 0.0)
            SetPrivateCurrentCfg(spp, new ShimmerConfig());

            // Start streaming (aggancia handler)
            spp.GetType().GetMethod("Start", BindingFlags.Instance | BindingFlags.Public)!.Invoke(spp, null);

            // Simula arrivo pacchetto dati
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
            // Campi EXG sempre presenti come double
            Assert.True(root.TryGetProperty("Exg1", out var _));
            Assert.True(root.TryGetProperty("Exg2", out var _));
            // Nessun blocco IMU se non abilitato
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

        [Fact]
        public void Start_DoesNot_DoubleSubscribe_When_Called_Twice()
        {
            var mac = "AA:BB:CC:DD:EE:03";
            var sent = new List<(string Mac, string Json)>();
            var spp = CreateSppSession(mac, (m, j) => sent.Add((m, j)));

            var core = new ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2("Stub", mac);
            spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, core);

            SetPrivateCurrentCfg(spp, new ShimmerConfig());

            // Start due volte: la seconda deve prima sganciare l’handler precedente
            var mStart = spp.GetType().GetMethod("Start", BindingFlags.Instance | BindingFlags.Public)!;
            mStart.Invoke(spp, null);
            mStart.Invoke(spp, null);

            // Un solo pacchetto → un solo broadcast (niente doppioni)
            var ev = new ShimmerAPI.CustomEventArgs(
                (int)ShimmerAPI.ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET,
                new ShimmerAPI.ObjectCluster());
            core.RaiseUi(ev);

            Assert.Single(sent);
        }

        [Fact]
        public void Start_Includes_Enabled_Blocks_And_LastKnown_Values()
        {
            var mac = "AA:BB:CC:DD:EE:04";
            var sent = new List<(string Mac, string Json)>();
            var spp = CreateSppSession(mac, (m, j) => sent.Add((m, j)));

            var core = new ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2("Stub", mac);
            spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(spp, core);

            // Abilita alcuni blocchi + setta last-known values manualmente
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

            // Start e invia un pacchetto
            spp.GetType().GetMethod("Start", BindingFlags.Instance | BindingFlags.Public)!.Invoke(spp, null);
            var ev = new ShimmerAPI.CustomEventArgs(
                (int)ShimmerAPI.ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET,
                new ShimmerAPI.ObjectCluster());
            core.RaiseUi(ev);

            Assert.Single(sent);
            using var doc = JsonDocument.Parse(sent[0].Json);
            var root = doc.RootElement;

            // exg_mode presente perché ExgMode != None
            Assert.True(root.TryGetProperty("exg_mode", out var exgModeProp));
            Assert.False(string.IsNullOrWhiteSpace(exgModeProp.GetString()));

            // lna, gyro presenti (wra/mag/press/… no perché non abilitati)
            Assert.True(root.TryGetProperty("lna", out _));
            Assert.True(root.TryGetProperty("gyro", out _));

            // vbatt presente con last-known
            Assert.True(root.TryGetProperty("vbatt", out var vb));
            Assert.Equal(3.7, vb.GetDouble(), 3);

            // ext presente con a6/a7/a15
            Assert.True(root.TryGetProperty("ext", out var ext));
            Assert.Equal(1.2, ext.GetProperty("a6").GetDouble(), 3);
            Assert.Equal(2.3, ext.GetProperty("a7").GetDouble(), 3);
            Assert.Equal(4.5, ext.GetProperty("a15").GetDouble(), 3);
        }


        // Stop behavior

        // --- Stop() behavior --------------------------------------------------------

        [Fact]
        public async Task Stop_Unsubscribes_Handler_NoFurtherBroadcasts()
        {
            // Arrange: session con broadcast “spiato”
            var sent = new List<(string mac, string json)>();
            var spp = CreateSppSession("01:23:45:67:89:AB", (m, j) => sent.Add((m, j)));

            // Apri connessione e avvia streaming (installa _handler)
            var mOpen = spp.GetType().GetMethod("OpenAsync", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mOpen);
            await (Task)mOpen!.Invoke(spp, null)!;

            var mStart = spp.GetType().GetMethod("Start", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mStart);
            mStart!.Invoke(spp, null);

            // Ricava _core e invia un pacchetto dati
            var fCore = spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fCore);
            var core = (ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2)fCore!.GetValue(spp)!;

            core.RaiseUi(new ShimmerAPI.CustomEventArgs(
                (int)ShimmerAPI.ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET,
                new ShimmerAPI.ObjectCluster()));

            Assert.NotEmpty(sent);            // riceviamo qualcosa prima dello stop
            var firstCount = sent.Count;

            // Act: Stop -> disiscrive _handler ed invia StopStreaming()
            var mStop = spp.GetType().GetMethod("Stop", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mStop);
            mStop!.Invoke(spp, null);

            // Prova a reinviare un pacchetto → non deve più arrivare nulla
            core.RaiseUi(new ShimmerAPI.CustomEventArgs(
                (int)ShimmerAPI.ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET,
                new ShimmerAPI.ObjectCluster()));

            // Assert
            Assert.Equal(firstCount, sent.Count);   // nessuna nuova emissione dopo Stop()
        }

        [Fact]
        public async Task Stop_When_NotStarted_Is_Idempotent_And_Safe()
        {
            // Arrange: apri ma NON chiamare Start()
            var sent = new List<(string mac, string json)>();
            var spp = CreateSppSession("AA:BB:CC:DD:EE:FF", (m, j) => sent.Add((m, j)));

            var mOpen = spp.GetType().GetMethod("OpenAsync", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mOpen);
            await (Task)mOpen!.Invoke(spp, null)!;

            // Act: Stop senza handler installato → non deve lanciare
            var mStop = spp.GetType().GetMethod("Stop", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mStop);
            mStop!.Invoke(spp, null);

            // E anche una seconda volta (idempotente)
            mStop!.Invoke(spp, null);

            // Invia pacchetto dati → nessun handler, quindi nessuna emissione
            var fCore = spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fCore);
            var core = (ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2)fCore!.GetValue(spp)!;

            core.RaiseUi(new ShimmerAPI.CustomEventArgs(
                (int)ShimmerAPI.ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET,
                new ShimmerAPI.ObjectCluster()));

            Assert.Empty(sent);
        }

        [Fact]
        public void Stop_When_CoreIsNull_DoesNotThrow()
        {
            // Arrange: session mai aperta (core == null)
            var spp = CreateSppSession("11:22:33:44:55:66");

            // Act + Assert: non deve eccepire
            var mStop = spp.GetType().GetMethod("Stop", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mStop);
            mStop!.Invoke(spp, null);
        }

        // Dispose behavior
        // --- Dispose() behavior ------------------------------------------------------

        [Fact]
        public async Task Dispose_StopsStreaming_Disconnects_Core_IsCleared()
        {
            // Arrange
            var sent = new List<(string mac, string json)>();
            var spp = CreateSppSession("DE:AD:BE:EF:00:01", (m, j) => sent.Add((m, j)));

            // Open + Start to install handler
            var mOpen = spp.GetType().GetMethod("OpenAsync", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mOpen);
            await (Task)mOpen!.Invoke(spp, null)!;

            var mStart = spp.GetType().GetMethod("Start", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mStart);
            mStart!.Invoke(spp, null);

            // Prendi un riferimento al core PRIMA del Dispose (così possiamo verificare lo stato dopo)
            var fCore = spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fCore);
            var core = (ShimmerSDK.Android.ShimmerLogAndStreamAndroidBluetoothV2)fCore!.GetValue(spp)!;

            // Verifica che arrivino pacchetti prima del Dispose
            core.RaiseUi(new ShimmerAPI.CustomEventArgs(
                (int)ShimmerAPI.ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET,
                new ShimmerAPI.ObjectCluster()));
            Assert.NotEmpty(sent);
            var before = sent.Count;

            // Act
            var mDispose = spp.GetType().GetMethod("Dispose", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mDispose);
            mDispose!.Invoke(spp, null);

            // Assert: core disconnesso e azzerato nel session object
            Assert.False(core.IsConnected());
            Assert.Null(fCore.GetValue(spp));

            // E il handler è stato disiscritto: nessun nuovo broadcast
            core.RaiseUi(new ShimmerAPI.CustomEventArgs(
                (int)ShimmerAPI.ShimmerBluetooth.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET,
                new ShimmerAPI.ObjectCluster()));
            Assert.Equal(before, sent.Count);
        }

        [Fact]
        public void Dispose_When_NotOpened_Is_Safe_And_NoThrow()
        {
            // Arrange: session mai aperta (core == null)
            var spp = CreateSppSession("DE:AD:BE:EF:00:02");

            // Act + Assert
            var mDispose = spp.GetType().GetMethod("Dispose", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mDispose);
            mDispose!.Invoke(spp, null);

            // core resta null
            var fCore = spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fCore);
            Assert.Null(fCore!.GetValue(spp));
        }

        [Fact]
        public async Task Dispose_Is_Idempotent()
        {
            // Arrange
            var spp = CreateSppSession("DE:AD:BE:EF:00:03");

            var mOpen = spp.GetType().GetMethod("OpenAsync", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mOpen);
            await (Task)mOpen!.Invoke(spp, null)!;

            var mDispose = spp.GetType().GetMethod("Dispose", BindingFlags.Instance | BindingFlags.Public);
            Assert.NotNull(mDispose);

            // Act: due chiamate consecutive
            mDispose!.Invoke(spp, null);
            mDispose!.Invoke(spp, null);

            // Assert: core nullo
            var fCore = spp.GetType().GetField("_core", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(fCore);
            Assert.Null(fCore!.GetValue(spp));
        }

        // SafeIdx behavior
        // --- SafeIdx behavior --------------------------------------------------------

        private static MethodInfo GetSafeIdxMethod(object spp)
        {
            var mi = spp.GetType().GetMethod("SafeIdx",
                BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return mi!;
        }

        [Fact]
        public void SafeIdx_Returns_Index_When_LabelExists()
        {
            var spp = CreateSppSession("11:22:33:44:55:66");
            var safeIdx = GetSafeIdxMethod(spp);

            var oc = new ShimmerAPI.ObjectCluster();
            var result = (int)safeIdx.Invoke(null, new object[] { oc, "__FIVE__", "CAL" })!;

            Assert.Equal(5, result);
        }

        [Fact]
        public void SafeIdx_Returns_MinusOne_When_LabelMissing()
        {
            var spp = CreateSppSession("AA:BB:CC:DD:EE:FF");
            var safeIdx = GetSafeIdxMethod(spp);

            var oc = new ShimmerAPI.ObjectCluster(); // GetIndex -> -1
            var result = (int)safeIdx.Invoke(null, new object[] { oc, "Unknown Label", "CAL" })!;

            Assert.Equal(-1, result);
        }

        [Fact]
        public void SafeIdx_Returns_MinusOne_When_GetIndex_Throws()
        {
            var spp = CreateSppSession("DE:AD:BE:EF:00:01");
            var safeIdx = GetSafeIdxMethod(spp);

            var oc = new ShimmerAPI.ObjectCluster(); // GetIndex lancerà su "__THROW__"
            var result = (int)safeIdx.Invoke(null, new object[] { oc, "__THROW__", "CAL" })!;

            Assert.Equal(-1, result);
        }

        // SafeGet behavior
        // --- SafeGet behavior --------------------------------------------------------

        private static System.Reflection.MethodInfo GetSafeGetMethod(object spp)
        {
            var mi = spp.GetType().GetMethod("SafeGet",
                BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return mi!;
        }

        [Fact]
        public void SafeGet_ReturnsNull_When_Index_Is_Negative()
        {
            var spp = CreateSppSession("AA:BB:CC:00:00:01");
            var safeGet = GetSafeGetMethod(spp);

            var oc = new ShimmerAPI.ObjectCluster();
            var res = safeGet.Invoke(null, new object[] { oc, -1 });

            Assert.Null(res);
        }

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

        [Fact]
        public void SafeGet_ReturnsNull_When_GetData_Throws()
        {
            var spp = CreateSppSession("AA:BB:CC:00:00:03");
            var safeGet = GetSafeGetMethod(spp);

            var oc = new ShimmerAPI.ObjectCluster();
            // 777: indice “speciale” che nello stub fa lanciare un’eccezione
            var res = safeGet.Invoke(null, new object[] { oc, 777 });

            Assert.Null(res);
        }

        // Val behavior
        // --- Val behavior ------------------------------------------------------------

        private static System.Reflection.MethodInfo GetValMethod(object spp)
        {
            var mi = spp.GetType().GetMethod("Val",
                BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return mi!;
        }

        private static ShimmerAPI.SensorData SD(object data) => new ShimmerAPI.SensorData(data);

        [Fact]
        public void Val_ReturnsNull_When_SensorData_Is_Null()
        {
            var spp = CreateSppSession("AA:BB:CC:10:00:01");
            var val = GetValMethod(spp);

            var res = val.Invoke(null, new object?[] { null });

            Assert.Null(res);
        }

        [Fact]
        public void Val_Converts_Int_To_Double()
        {
            var spp = CreateSppSession("AA:BB:CC:10:00:02");
            var val = GetValMethod(spp);

            var sd = SD(5); // int
            var res = (double?)val.Invoke(null, new object?[] { sd });

            Assert.Equal(5.0, res);
        }

        [Fact]
        public void Val_Converts_Double_To_Double_Unchanged()
        {
            var spp = CreateSppSession("AA:BB:CC:10:00:03");
            var val = GetValMethod(spp);

            var sd = SD(3.25); // double
            var res = (double?)val.Invoke(null, new object?[] { sd });

            Assert.Equal(3.25, res);
        }

        [Fact]
        public void Val_Converts_IntegerString_To_Double()
        {
            var spp = CreateSppSession("AA:BB:CC:10:00:04");
            var val = GetValMethod(spp);

            var sd = SD("42"); // string numerica “safe” (niente problemi di cultura)
            var res = (double?)val.Invoke(null, new object?[] { sd });

            Assert.Equal(42.0, res);
        }

        [Fact]
        public void Val_ReturnsNull_On_NonNumeric_String()
        {
            var spp = CreateSppSession("AA:BB:CC:10:00:05");
            var val = GetValMethod(spp);

            var sd = SD("not-a-number");
            var res = (double?)val.Invoke(null, new object?[] { sd });

            Assert.Null(res);
        }

        [Fact]
        public void Val_ReturnsNull_On_Object_Not_Convertible()
        {
            var spp = CreateSppSession("AA:BB:CC:10:00:06");
            var val = GetValMethod(spp);

            var sd = SD(new object()); // Convert.ToDouble lancerà
            var res = (double?)val.Invoke(null, new object?[] { sd });

            Assert.Null(res);
        }

        // IsConnectedState behavior

        // --- IsConnectedState behavior ----------------------------------------------

        private static System.Reflection.MethodInfo GetIsConnectedMethod(object spp)
        {
            var mi = spp.GetType().GetMethod("IsConnectedState",
                BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return mi!;
        }

        [Fact]
        public void IsConnectedState_ReturnsFalse_On_Null()
        {
            var spp = CreateSppSession("AA:BB:00:00:01");
            var m = GetIsConnectedMethod(spp);

            var res = (bool)m.Invoke(null, new object?[] { null })!;
            Assert.False(res);
        }

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

        [Fact]
        public void IsConnectedState_Uses_ToString_Fallback()
        {
            var spp = CreateSppSession("AA:BB:00:00:05");
            var m = GetIsConnectedMethod(spp);

            var obj = new { Status = "CONNECTED" }; // ToString() => "{ Status = CONNECTED }"
            var res = (bool)m.Invoke(null, new object?[] { obj })!;
            Assert.True(res);
        }

        [Fact]
        public void IsConnectedState_ReturnsFalse_On_NonConvertible_Object()
        {
            var spp = CreateSppSession("AA:BB:00:00:06");
            var m = GetIsConnectedMethod(spp);

            var obj = new object(); // ToString() -> "System.Object" (non contiene CONNECTED)
            var res = (bool)m.Invoke(null, new object?[] { obj })!;
            Assert.False(res);
        }


        // TryIdx behavior
        [Fact]
        public void TryIdx_Returns_FirstMatchingIndex()
        {
            // OC finto che risponde solo a ("Gyroscope X","CAL") ⇒ indice 5
            var oc = new ShimmerAPI.ObjectClusterMock()
                .When("Gyroscope X", "CAL", 5);

            // candidati: il 1° non esiste, il 2° è quello giusto ⇒ deve tornare 5
            int i = InvokeTryIdx(oc,
                ("WR Accel X", "CAL"),
                ("Gyroscope X", "CAL"),
                ("Magnetometer X", "CAL"));

            Assert.Equal(5, i);
        }

        [Fact]
        public void TryIdx_Returns_MinusOne_When_NoCandidatesMatch()
        {
            var oc = new ShimmerAPI.ObjectClusterMock(); // nessun mapping
            int i = InvokeTryIdx(oc, ("Foo", "CAL"), ("Bar", "RAW"));
            Assert.Equal(-1, i);
        }

        [Fact]
        public void TryIdx_Stops_At_First_Match()
        {
            var oc = new ShimmerAPI.ObjectClusterMock()
                .When("A", "CAL", 2)
                .When("B", "CAL", 7); // non dovrebbe arrivare a qui

            int i = InvokeTryIdx(oc, ("A", "CAL"), ("B", "CAL"));
            Assert.Equal(2, i);
        }

        [Fact]
        public void TryIdx_Swallows_Internal_GetIndex_Errors()
        {
            var oc = new ShimmerAPI.ObjectClusterMock()
                .ThrowOn("Bad", "CAL");

            int i = InvokeTryIdx(oc, ("Bad", "CAL"), ("Good", "CAL"));
            // il primo cand solleva, il secondo non esiste ⇒ -1
            Assert.Equal(-1, i);
        }

        // --- Helpers test-only ---

        // Richiama via reflection il tuo metodo privato statico TryIdx(ObjectCluster,(string,string)[])
        private static int InvokeTryIdx(object oc, params (string name, string fmt)[] cands)
        {
            var spp = CreateSppSession("00:11:22:33:44:55"); // come negli altri test
            var mi = spp.GetType().GetMethod("TryIdx",
                BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(mi);
            return (int)mi!.Invoke(spp, new object?[] { oc, cands })!;
        }

        // LooksLikeShimmer behavior

        // Helper: invoca il metodo statico privato LooksLikeShimmer(string?, string?)
        private static bool CallLooksLikeShimmer(string? name, string? mac)
        {
            var t = typeof(WsBridgeManager); // <-- se il metodo è in un altro tipo, cambia qui
            var m = t.GetMethod("LooksLikeShimmer", BindingFlags.Static | BindingFlags.NonPublic);
            Assert.NotNull(m);
            return (bool)m!.Invoke(null, new object?[] { name, mac })!;
        }

        [Theory]
        // Match su nome: contiene "Shimmer"
        [InlineData("Shimmer3", null)]
        [InlineData("my_shimmer_node", null)]
        // Match su nome: prefissi RN*
        [InlineData("RNBT-1234", null)]
        [InlineData("RN42-ABCD", null)]
        [InlineData("RN-42-ABCD", null)]
        // Match su MAC OUI 00:06:66 (solo con ':')
        [InlineData(null, "00:06:66:AA:BB:CC")]
        [InlineData("Other", "00:06:66:FF:EE:DD")]
        // Case-insensitive
        [InlineData("shimmer", null)]
        [InlineData(null, "00:06:66:aa:bb:cc")]
        public void LooksLikeShimmer_ReturnsTrue_OnKnownPatterns(string? name, string? mac)
        {
            Assert.True(CallLooksLikeShimmer(name, mac));
        }

        [Theory]
        // Nessun match
        [InlineData(null, null)]
        [InlineData("", "")]
        [InlineData("Device", null)]
        // MAC con separatori diversi o nessun separatore -> non matcha (implementazione attuale usa StartsWith con ':')
        [InlineData(null, "00-06-66-AA-BB-CC")]
        [InlineData(null, "000666AABBCC")]
        // MAC con spazio iniziale e nessun match su name -> non matcha
        [InlineData("Other", " 00:06:66:AA:BB:CC")]
        // Nome che non contiene i pattern previsti
        [InlineData("SHMR-Device", null)]
        public void LooksLikeShimmer_ReturnsFalse_OnUnknownPatterns(string? name, string? mac)
        {
            Assert.False(CallLooksLikeShimmer(name, mac));
        }

        [Theory]
        // Controllo che la ricerca sia case-insensitive e che "Contains" funzioni anche con spazi
        [InlineData("  Shimmer  ", null, true)]
        [InlineData("  rnBt-XYZ", null, false)]
        [InlineData(" device ", " 00:06:66:11:22:33", false)] // spazio nel MAC -> niente match
        public void LooksLikeShimmer_MixedCasesAndSpaces(string? name, string? mac, bool expected)
        {
            Assert.Equal(expected, CallLooksLikeShimmer(name, mac));
        }

        //  SafeSend behavior

        private static object NewBridge()
        {
            var t = typeof(WsBridgeManager);

            // 1) prova un costruttore (pubblico o non pubblico) senza argomenti
            var ctor0 = t.GetConstructors(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic)
                         .FirstOrDefault(c => c.GetParameters().Length == 0);
            if (ctor0 != null)
                return ctor0.Invoke(Array.Empty<object?>());

            // 2) altrimenti usa il costruttore “più semplice” e passa default ai parametri
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

                // Valori di default “sicuri” per i test
                if (pt == typeof(string)) args[i] = string.Empty;
                else if (pt.IsValueType) args[i] = Activator.CreateInstance(pt); // es. 0, false, …
                else args[i] = null;                          // ref-type
            }

            return anyCtor.Invoke(args);
        }

        // Helper: invoca SafeSend (privato) via reflection
        private static Task InvokeSafeSend(object bridge, Guid clientId, string json)
        {
            var m = bridge.GetType().GetMethod("SafeSend", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(m);
            return (Task)m!.Invoke(bridge, new object[] { clientId, json })!;
        }

        // Helper: imposta il campo _ws (privato) del bridge
        private static void SetWsServer(object bridge, WatsonWsServer? server)
        {
            var f = bridge.GetType().GetField("_ws", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.NotNull(f);
            f!.SetValue(bridge, server);
        }

        // (opzionale) Helper: imposta il logger privato Log, così se il send fallisce possiamo verificare che non lanci
        private static void SetLog(object bridge, Action<string>? log)
        {
            var f = bridge.GetType().GetField("Log", BindingFlags.Instance | BindingFlags.NonPublic);
            if (f != null) f.SetValue(bridge, log);
        }

        [Fact]
        public async Task SafeSend_WhenServerIsNull_CompletesWithoutThrowing()
        {
            var bridge = NewBridge();          // usa il factory senza FormatterServices
            SetWsServer(bridge, null);         // _ws == null
            SetLog(bridge, _ => { });          // opzionale

            var id = Guid.NewGuid();
            var payload = "{\"ok\":true}";

            await InvokeSafeSend(bridge, id, payload); // non deve lanciare
        }

        [Fact]
        public async Task SafeSend_DelegatesToServerSendAsync_AndRecordsMessage()
        {
            var bridge = NewBridge();

            var server = new WatsonWsServer("127.0.0.1", 9000, false);
            SetWsServer(bridge, server);

            var id = Guid.NewGuid();
            var payload = "{\"type\":\"ping\"}";

            await InvokeSafeSend(bridge, id, payload);

            // Lo stub espone la lista "Sent": verifichiamo che il messaggio sia uscito
            Assert.Contains(server.Sent, t => t.clientId == id && t.message == payload);
        }

        // GetLocalIp behavior

        private static string CallGetLocalIp(Activity? act)
        {
            var t = typeof(WsBridgeManager);
            var m = t.GetMethod("GetLocalIp", BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(m);
            return (string)m!.Invoke(null, new object?[] { act })!;
        }

        [Fact]
        public void GetLocalIp_Returns_0_0_0_0_When_Activity_Is_Null()
        {
            var ip = CallGetLocalIp(null);
            Assert.Equal("0.0.0.0", ip);
        }

        [Fact]
        public void GetLocalIp_Returns_0_0_0_0_When_ConnectionInfo_Is_Null()
        {
            var act = new Activity();
            var wm = (WifiManager)act.GetSystemService(Activity.WifiService)!;
            wm.ConnectionInfo = null; // forza il ramo null

            var ip = CallGetLocalIp(act);
            Assert.Equal("0.0.0.0", ip);
        }

        [Fact]
        public void GetLocalIp_Parses_LittleEndian_Int_To_DottedQuad()
        {
            // Lo stub WifiInfo default è 192.168.1.42 (codificato little-endian)
            var act = new Activity();
            var ip = CallGetLocalIp(act);
            Assert.Equal("192.168.1.42", ip);
        }

        [Theory]
        // Verifica alcune combinazioni note: (A.B.C.D) -> (D<<24 | C<<16 | B<<8 | A)
        [InlineData(10, 0, 0, 1)]       // 10.0.0.1
        [InlineData(172, 16, 5, 200)]   // 172.16.5.200
        [InlineData(192, 168, 100, 2)]  // 192.168.100.2
        public void GetLocalIp_Converts_IntCorrectly(int a, int b, int c, int d)
        {
            // compone l'int come fa Android (little-endian nei bit)
            int androidInt = (d << 24) | (c << 16) | (b << 8) | a;

            var act = new Activity();
            var wm = (WifiManager)act.GetSystemService(Activity.WifiService)!;
            wm.ConnectionInfo!.IpAddress = androidInt;

            var ip = CallGetLocalIp(act);
            Assert.Equal($"{a}.{b}.{c}.{d}", ip);
        }

        // GetString behavior

        // Helper: invoca il metodo privato statico GetString(ArraySegment<byte>)
        private static string CallGetString(ArraySegment<byte> seg)
        {
            var t = typeof(WsBridgeManager); // cambia tipo se GetString è in un altro
            var m = t.GetMethod("GetString", BindingFlags.NonPublic | BindingFlags.Static);
            Assert.NotNull(m);
            return (string)m!.Invoke(null, new object?[] { seg })!;
        }

        [Fact]
        public void GetString_EmptySegment_ReturnsEmpty()
        {
            var seg = new ArraySegment<byte>(Array.Empty<byte>(), 0, 0);
            var s = CallGetString(seg);
            Assert.Equal(string.Empty, s);
        }

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

        [Fact]
        public void GetString_DecodesPlainAscii()
        {
            var txt = "hello-world";
            var seg = new ArraySegment<byte>(Encoding.UTF8.GetBytes(txt));
            var s = CallGetString(seg);
            Assert.Equal(txt, s);
        }

        [Fact]
        public void GetString_DecodesOnlySelectedSegment()
        {
            // payload completo: "xxxx" + "ciao🌟" + "yyyy"
            var prefix = Encoding.UTF8.GetBytes("xxxx");
            var middle = Encoding.UTF8.GetBytes("ciao🌟"); // include char multibyte
            var suffix = Encoding.UTF8.GetBytes("yyyy");

            var buffer = new byte[prefix.Length + middle.Length + suffix.Length];
            Buffer.BlockCopy(prefix, 0, buffer, 0, prefix.Length);
            Buffer.BlockCopy(middle, 0, buffer, prefix.Length, middle.Length);
            Buffer.BlockCopy(suffix, 0, buffer, prefix.Length + middle.Length, suffix.Length);

            // segment che punta solo a "ciao🌟"
            var seg = new ArraySegment<byte>(buffer, prefix.Length, middle.Length);

            var s = CallGetString(seg);
            Assert.Equal("ciao🌟", s);
        }
    }
}
