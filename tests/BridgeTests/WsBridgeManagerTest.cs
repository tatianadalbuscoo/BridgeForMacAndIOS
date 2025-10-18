using System;
using System.Linq;
using Xunit;
using ShimmerBridgeMangager;
using System.Text.Json;
using System.Threading.Tasks;
using ShimmerAPI;



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




    }
}
