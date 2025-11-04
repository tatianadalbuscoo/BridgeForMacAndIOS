/*
Main Android Activity for Shimmer Bridge.
Lists paired Shimmer devices, lets users pick sensors, and starts/stops streaming via WS.
Handles runtime permissions, scanning/classification (IMU/EXG/OFF), and status UI.
Builds per-device ShimmerConfig (EXG mode only for EXG) and debounces reconfigs.
Cleans up timers and stops the WebSocket bridge on destroy.
*/


using Android.Bluetooth;
using Android.OS;
using Android.Content.PM;
using Android.Views;
using STimers = System.Timers;
using ShimmerBridgeMangager;
using ShimmerBridgeScan;
using Android.Graphics;
using Android.Graphics.Drawables;
using AndroidResource = ShimmerAndroidBridge.Resource;


namespace ShimmerBridge
{

    /// <summary>
    /// Main Android Activity for the Shimmer Bridge app.
    /// Builds the UI, handles Bluetooth permissions/scan, and manages
    /// device selection, configuration, and start/stop of streaming via WsBridgeManager.
    /// </summary>

    // Main entry Activity for the app.
    [Activity(
        Label = "Shimmer Bridge",
        Theme = "@style/AppTheme",
        MainLauncher = true,
        Name = "ShimmerBridge.MainActivity"
    )]

    public class MainActivity : Activity
    {

        // Container where device cards (rows) are added
        LinearLayout _deviceGroup = null!;

        // Status label shown at the bottom/top of the screen
        TextView _status = null!;

        // Start/Stop streaming buttons
        Button _btnStart = null!, _btnStop = null!;

        // Manual "Refresh" scan button
        Button _btnScan = null!;

        // WebSocket bridge to the streaming server
        readonly WsBridgeManager _ws = new WsBridgeManager();

        // UI state per device (sensors + connect toggle)
        readonly List<DeviceUi> _devices = new();

        // Per-MAC debounce timers
        readonly Dictionary<string, STimers.Timer> _debouncers = new(StringComparer.OrdinalIgnoreCase);

        // Helper to scan and classify Shimmer devices
        ShimmerScanManager _scanner = null!;

        // Logcat tag
        const string TAG = "ShimmerBridgeUI";

        // Guard to prevent concurrent scans
        volatile bool _isScanning = false;

        // Modal progress dialog shown during scan/connect
        AlertDialog? _scanDialog = null;

        // Last known type per device (IMU/EXG/Unknown/OFF)
        Dictionary<string, ShimmerScanManager.DeviceType> _lastTypes =
            new(StringComparer.OrdinalIgnoreCase);


        /// <summary>
        /// Converts density-independent pixels (dp) to physical pixels for this device.
        /// </summary>
        /// <param name="dp">Value in dp (density-independent pixels).</param>
        /// <returns>Equivalent pixel value rounded to the nearest int.</returns>
        int Dp(int dp)
        {
            var density = Resources?.DisplayMetrics?.Density ?? 1f;
            return (int)(dp * density + 0.5f);
        }


        /// <summary>
        /// Checks whether at least one sensor is enabled on the given device card.
        /// </summary>
        /// <param name="d">The device UI to inspect.</param>
        /// <returns><c>true</c> if any sensor checkbox is selected; otherwise, <c>false</c>.</returns>
        static bool AnySensorEnabled(DeviceUi d) =>
            d.LnAcc.Checked || d.WrAcc.Checked || d.Gyro.Checked ||
            d.Mag.Checked || d.Press.Checked || d.Batt.Checked ||
            d.A6.Checked || d.A7.Checked || d.A15.Checked;


        /// <summary>
        /// Initializes the Activity: inflates the layout, binds views and handlers,
        /// sets up logging and the scanner, requests runtime permissions,
        /// and performs the initial device population plus a first scan.
        /// </summary>
        /// <param name="savedInstanceState">
        /// If non-null, the activity is being re-initialized after previously being shut down;
        /// contains the most recent data supplied/>.
        /// </param>
        protected override void OnCreate(Bundle? savedInstanceState)
        {
            base.OnCreate(savedInstanceState);

            try
            {
                SetContentView(AndroidResource.Layout.activity_main);

                // Strictly bind required views (throws if missing)
                _deviceGroup = RequireView<LinearLayout>(AndroidResource.Id.deviceGroup, "deviceGroup");
                _status = RequireView<TextView>(AndroidResource.Id.txtStatus, "txtStatus");
                _btnStart = RequireView<Button>(AndroidResource.Id.btnStart, "btnStart");
                _btnStop = RequireView<Button>(AndroidResource.Id.btnStop, "btnStop");

                // Pipe WsBridge logs to UI
                _ws.Log += s => RunOnUiThread(() => _status.Text = "Status: " + s);

                // Primary actions
                _btnStart.Click += async (_, __) => await ConnectAndStartAsync();
                _btnStop.Click += async (_, __) => await StopAllWithNoticeAsync();

                // BT scanner
                _scanner = new ShimmerScanManager(this);

                EnsureRuntimePermissions();     // BT/Location on Android 12+
                AddScanControls();              // Add "Refresh" row
                PopulateBondedDevices();        // Render paired device cards
                _ = StartScanNow();             // Initial scan + type badges
            }
            catch (Exception ex)
            {
                Android.Util.Log.Error(TAG, "Init error:\n" + ex);
                var toast = Toast.MakeText(this, ex.Message ?? "Unexpected error", ToastLength.Long);
                toast?.Show();
            }
        }


        /// <summary>
        /// Finds a view by its resource id and casts it to <typeparamref name="T"/>.
        /// Throws if the view is missing to fail fast on layout bugs.
        /// </summary>
        /// <typeparam name="T">Expected view type (e.g., TextView, Button).</typeparam>
        /// <param name="id">Resource id from the generated Resource.Id.*</param>
        /// <param name="name">Friendly name used in the exception message.</param>
        /// <returns>The view cast to <typeparamref name="T"/>.</returns>
        T RequireView<T>(int id, string name) where T : class
        {
            var v = FindViewById(id) as T;   // Safe cast; null if type/id mismatch
            if (v == null) throw new NullReferenceException($"View '{name}' not found in activity_main.axml");
            return v;
        }


        /// <summary>
        /// Requests runtime permissions required for Bluetooth scanning/connecting
        /// (and location on older Android). No-ops if already granted.
        /// </summary>
        void EnsureRuntimePermissions()
        {
            if (Build.VERSION.SdkInt >= BuildVersionCodes.S)     // Android 12 (API 31)+
            {
                var needed = new[]
                {
                    Android.Manifest.Permission.BluetoothScan,
                    Android.Manifest.Permission.BluetoothConnect,
                    Android.Manifest.Permission.AccessFineLocation  // still needed by some stacks
                }.Where(p => CheckSelfPermission(p) != Permission.Granted).ToArray();

                if (needed.Length > 0) RequestPermissions(needed, 42);
            }
            else
            {

                // Pre-Android 12: Bluetooth scan requires fine location
                if (CheckSelfPermission(Android.Manifest.Permission.AccessFineLocation) != Permission.Granted)
                    RequestPermissions(new[] { Android.Manifest.Permission.AccessFineLocation }, 42);
            }
        }


        /// <summary>
        /// Adds a thin top row with a manual "Refresh" scan button to the device list.
        /// </summary>
        void AddScanControls()
        {

            // Horizontal row container with padding
            var row = new LinearLayout(this)
            {
                Orientation = Orientation.Horizontal
            };
            row.SetPadding(Dp(6), Dp(6), Dp(6), Dp(6));

            // "Refresh" button triggers a new scan
            _btnScan = new Button(this) { Text = "Refresh" };
            _btnScan.Click += async (_, __) => await StartScanNow();

            // Add the button to the row
            row.AddView(_btnScan, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WrapContent, ViewGroup.LayoutParams.WrapContent));

            // Insert the row as the first child of the device group
            _deviceGroup.AddView(row, 0);
        }


        /// <summary>
        /// Renders cards for already-paired Shimmer devices (RN-42/IMU/EXG).
        /// Shows a hint if no adapter or no paired devices are found.
        /// </summary>
        void PopulateBondedDevices()
        {

            // Clear previous cards, but keep the top scan row at index 0
            while (_deviceGroup.ChildCount > 1)
                _deviceGroup.RemoveViewAt(1);

            _devices.Clear();

            var bt = BluetoothAdapter.DefaultAdapter;
            if (bt == null)
            {
                Android.Util.Log.Warn(TAG, "BluetoothAdapter.DefaultAdapter == null (emulator?)");
                _status.Text = "Status: no Bluetooth adapter (use a physical device)";
                return;     // Cannot proceed without a real adapter
            }

            // Get paired devices with a non-null name
            var paired = bt.BondedDevices?.Where(d => d?.Name != null).ToList() ?? new List<BluetoothDevice>();
            if (paired.Count == 0)
            {

                // User guidance: how to pair Shimmer/RN-42
                var tv = new TextView(this) { Text = "No paired devices. Pair RN-42/Shimmer in Android settings (PIN 1234)." };
                _deviceGroup.AddView(tv);
                return;
            }

            // Keep only devices that look like Shimmer; sort for stable UI
            foreach (var d in paired
                     .Where(d => ShimmerScanManager.LooksLikeShimmer(d.Name, d.Address))
                     .OrderBy(d => d.Name ?? d.Address))
            {

                // Build and attach a card; keep a reference for later interactions
                var ui = BuildDeviceCard(d);
                _devices.Add(ui);
                _deviceGroup.AddView(ui.Root);
            }
        }


        /// <summary>
        /// Builds a UI card for a paired Bluetooth device, including sensor toggles,
        /// a Connect checkbox, a type badge, and (for EXG) a mode selector.
        /// </summary>
        /// <param name="d">The paired <see cref="BluetoothDevice"/> to render into a card.</param>
        /// <returns>
        /// A <see cref="DeviceUi"/> handle referencing the created views (card root, controls, badge),
        /// used later to read state and apply updates.
        /// </returns>
        DeviceUi BuildDeviceCard(BluetoothDevice d)
        {

            // --- Outer card container -------------------------------------------------
            var card = new LinearLayout(this) { Orientation = Orientation.Vertical };
            card.SetPadding(Dp(12), Dp(12), Dp(12), Dp(12));
            var lp = new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MatchParent, ViewGroup.LayoutParams.WrapContent)
            {
                TopMargin = Dp(8),
                BottomMargin = Dp(8)
            };
            card.LayoutParameters = lp;
            card.SetBackgroundColor(Color.ParseColor("#EFE9E3")); // soft beige

            // --- Title row: [Name/MAC]  [Connect]  [Badge] ----------------------------
            var titleRow = new LinearLayout(this)
            {
                Orientation = Orientation.Horizontal
            };
            titleRow.SetGravity(GravityFlags.CenterVertical);

            var title = new TextView(this)
            {
                Text = $"{d.Name} [{d.Address}]",
                TextSize = 18
            };
            title.SetTypeface(title.Typeface, TypefaceStyle.Bold);

            var cbConnect = new CheckBox(this) { Text = "Connect", Checked = false };

            var badge = MakeBadgeView("?");  // Device type badge: IMU/EXG/?/OFF

            // Layout: title expands (weight=1), then Connect, then Badge
            titleRow.AddView(title, new LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WrapContent, 1f));
            titleRow.AddView(cbConnect, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WrapContent, ViewGroup.LayoutParams.WrapContent)
            {
                RightMargin = Dp(8)
            });
            titleRow.AddView(badge, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WrapContent, ViewGroup.LayoutParams.WrapContent)
            {
                LeftMargin = Dp(8)
            });

            card.AddView(titleRow);

            // --- Subtitle --------------------------------------------------------------
            var sub = new TextView(this) { Text = "Select sensors for this device" };
            sub.SetPadding(0, Dp(6), 0, Dp(6));
            card.AddView(sub);

            // --- Sensor checkboxes (default ON) ---------------------------------------
            CheckBox NewCb(string text) => new CheckBox(this) { Text = text, Checked = true };

            var lnAcc = NewCb("Low-noise accelerometer");
            var wrAcc = NewCb("Wide-range accelerometer");
            var gyro = NewCb("Gyroscope");
            var mag = NewCb("Magnetometer");
            var press = NewCb("Pressure & Temperature");
            var batt = NewCb("Battery");
            var a6 = NewCb("Ext A6");
            var a7 = NewCb("Ext A7");
            var a15 = NewCb("Ext A15");

            card.AddView(lnAcc);
            card.AddView(wrAcc);
            card.AddView(gyro);
            card.AddView(mag);
            card.AddView(press);
            card.AddView(batt);
            card.AddView(a6);
            card.AddView(a7);
            card.AddView(a15);

            // --- EXG mode selector (shown only for EXG devices) -----------------------
            var exgRow = new LinearLayout(this) 
            { 
                Orientation = Orientation.Horizontal, 
                Visibility = ViewStates.Gone    // Toggled visible when type == EXG
            };
            var lblExg = new TextView(this) { Text = "EXG mode: " };
            lblExg.SetPadding(0, Dp(8), Dp(8), Dp(8));

            var rg = new RadioGroup(this) { Orientation = Orientation.Horizontal };

            // Explicit IDs so we can call rg.Check
            var rbECG = new RadioButton(this) { Text = "ECG", Id = View.GenerateViewId() };
            var rbEMG = new RadioButton(this) { Text = "EMG", Id = View.GenerateViewId() };
            var rbTest = new RadioButton(this) { Text = "EXG Test", Id = View.GenerateViewId() };
            var rbResp = new RadioButton(this) { Text = "Respiration", Id = View.GenerateViewId() };

            rg.AddView(rbECG);
            rg.AddView(rbEMG);
            rg.AddView(rbTest);
            rg.AddView(rbResp);

            exgRow.AddView(lblExg);
            exgRow.AddView(rg);
            card.AddView(exgRow);

            // Handle for later access (connect toggle, sensors, badge, EXG row/group)
            var ui = new DeviceUi(card, d.Name ?? "?", d.Address ?? "?", cbConnect,
                                  lnAcc, wrAcc, gyro, mag, press, batt, a6, a7, a15,
                                  badge, exgRow, rg);

            // Default EXG mode to ECG (user can change later)
            rg.Check(rbECG.Id);
            ui.SelectedExgMode = "ECG";

            // --- Debounced reconfiguration when sensor checkboxes change --------------
            // Only triggers if the device is connected to avoid noisy reconfigs.
            void Hook(CheckBox cb)
            {
                cb.CheckedChange += (s, e) =>
                {
                    if (!ui.Connect.Checked) return;
                    DebouncedReconfigure(ui.Mac, () => BuildConfigFromUi(ui));
                };
            }
            Hook(lnAcc); Hook(wrAcc); Hook(gyro); Hook(mag); Hook(press); Hook(batt); Hook(a6); Hook(a7); Hook(a15);

            // --- EXG mode selection ---------------
            rg.CheckedChange += (s, e) =>
            {
                var id = rg.CheckedRadioButtonId;
                var rb = rg.FindViewById<RadioButton>(id);
                ui.SelectedExgMode = rb?.Text ?? "ECG";
            };

            return ui;
        }


        /// <summary>
        /// Builds a <see cref="ShimmerConfig"/> from the current UI state of a device card.
        /// EXG mode is NOT set here
        /// (it is applied later only for EXG devices). SamplingRate is left null by default.
        /// </summary>
        /// <param name="u">UI handle for the device (checkboxes and state).</param>
        /// <returns>The configuration to send to the bridge/server.</returns>
        ShimmerConfig BuildConfigFromUi(DeviceUi u) => new ShimmerConfig
        {
            EnableLowNoiseAccelerometer = u.LnAcc.Checked,
            EnableWideRangeAccelerometer = u.WrAcc.Checked,
            EnableGyroscope = u.Gyro.Checked,
            EnableMagnetometer = u.Mag.Checked,
            EnablePressureTemperature = u.Press.Checked,
            EnableBattery = u.Batt.Checked,
            EnableExtA6 = u.A6.Checked,
            EnableExtA7 = u.A7.Checked,
            EnableExtA15 = u.A15.Checked,
            SamplingRate = null
        };


        /// <summary>
        /// Debounces reconfiguration requests per device MAC. If the user toggles multiple
        /// checkboxes quickly, only the last change within the debounce window triggers.
        /// </summary>
        /// <param name="mac">Device MAC address used as the debounce key.</param>
        /// <param name="build">Factory that reads current UI and returns a fresh <see cref="ShimmerConfig"/>.</param>
        void DebouncedReconfigure(string mac, Func<ShimmerConfig> build)
        {

            // Cancel and dispose any pending debounce timer for this MAC
            if (_debouncers.TryGetValue(mac, out var t))
            {
                try { t.Stop(); t.Dispose(); } catch { }
            }

            // Single-shot timer: fires once after 250 ms of inactivity
            var timer = new STimers.Timer(250) { AutoReset = false };
            timer.Elapsed += async (_, __) =>
            {
                try
                {
                    var cfg = build();                      // Build config at execution time (latest UI state)
                    await _ws.UpdateConfigAsync(mac, cfg);  // If mask=0, server will stop stream
                    RunOnUiThread(() => _status.Text = $"Status: reconfigured {mac}");
                }
                catch (Exception ex)
                {
                    RunOnUiThread(() => _status.Text = $"Status: reconfig error {ex.Message}");
                }
            };

            // Replace previous timer and start a fresh debounce window
            _debouncers[mac] = timer;
            timer.Start();
        }


        /// <summary>
        /// Shows a non-cancelable modal progress dialog with an indeterminate spinner.
        /// Closes (Dismiss) any previous dialog instance before creating a new one,
        /// then stores the new dialog for later dismissal.
        /// </summary>
        /// <param name="title">Dialog title (defaults to "Scanning…").</param>
        void ShowScanDialog(string title = "Scanning…")
        {
            try { _scanDialog?.Dismiss(); } catch { }

            var pb = new ProgressBar(this!) { Indeterminate = true };   

            var builder = new Android.App.AlertDialog.Builder(this!);
            builder.SetTitle(title ?? "Scanning…");
            builder.SetView(pb);
            builder.SetCancelable(false);

            _scanDialog = builder.Create();   
            _scanDialog?.Show();             
        }


        /// <summary>
        /// Hides and releases the modal progress dialog if present.
        /// </summary>
        void HideScanDialog()
        {
            try { _scanDialog?.Dismiss(); } catch { }
            _scanDialog = null;
        }


        /// <summary>
        /// Performs a guarded Bluetooth scan, updates each device card with its detected type,
        /// and shows a modal spinner while scanning. No-ops if a scan is already in progress.
        /// </summary>
        async Task StartScanNow()
        {

            // Prevent concurrent scans (double-taps)
            if (_isScanning) return;
            _isScanning = true;

            try
            {
                _btnScan.Enabled = false;               // Disable manual scan button during work
                ShowScanDialog("Scanning devices…");    // Modal spinner

                _status.Text = "Status: scanning…";
                var res = await _scanner.ScanAsync(TimeSpan.FromSeconds(7));    // Do the scan

                // Map MAC -> Type for visible devices (IMU/EXG/Unknown)
                _lastTypes = res.Visible.ToDictionary(e => e.Mac, e => e.Type, StringComparer.OrdinalIgnoreCase);

                // Set of MACs that were explicitly seen as "off"
                var offSet = new HashSet<string>(res.Off.Select(e => e.Mac), StringComparer.OrdinalIgnoreCase);

                // Update each existing UI card with the latest classification
                foreach (var ui in _devices)
                {
                    ShimmerScanManager.DeviceType type;
                    if (_lastTypes.TryGetValue(ui.Mac, out var t))
                        type = t;   // Actual detected type (IMU/EXG/Unknown)
                    else if (offSet.Contains(ui.Mac))
                        type = ShimmerScanManager.DeviceType.DeviceOff; // Was seen as OFF
                    else
                        type = ShimmerScanManager.DeviceType.Unknown;   // Not observed

                    ApplyTypeToUi(ui, type);     // Badge + EXG row visibility
                }

                _status.Text = "Status: scan complete";
            }
            catch (Exception ex)
            {
                _status.Text = $"Status: scan error {ex.Message}";
            }
            finally
            {
                HideScanDialog();           // Close spinner
                _btnScan.Enabled = true;    // Re-enable "Refresh"
                _isScanning = false;        // Re-enable "Refresh"
            }
        }


        /// <summary>
        /// Applies the detected device type to the UI: updates the badge and toggles EXG controls.
        /// </summary>
        /// <param name="ui">Target device card UI handle.</param>
        /// <param name="type">Detected device type (IMU/EXG/Unknown/DeviceOff).</param>
        void ApplyTypeToUi(DeviceUi ui, ShimmerScanManager.DeviceType type)
        {
            ui.Type = type;                 // Persist latest classification
            SetBadge(ui.Badge, type);       // Update colored badge text/background
            ui.ExgRow.Visibility =          // Show EXG mode selector only for EXG devices 
                type == ShimmerScanManager.DeviceType.EXG ? ViewStates.Visible : ViewStates.Gone;
            // Hide the Connect "box" if OFF or Unknown
            ui.Connect.Visibility =
                (type == ShimmerScanManager.DeviceType.IMU || type == ShimmerScanManager.DeviceType.EXG)
                ? ViewStates.Visible
                : ViewStates.Gone;

            // Remove the checkmark
            if (ui.Connect.Visibility == ViewStates.Gone && ui.Connect.Checked)
                ui.Connect.Checked = false;

        }


        /// <summary>
        /// Creates a small rounded badge TextView with the given text and a neutral background.
        /// Caller should later recolor it via <see cref="SetBadge"/>.
        /// </summary>
        /// <param name="text">Initial text to display inside the badge.</param>
        /// <returns>A styled <see cref="TextView"/> suitable for status/type badges.</returns>
        TextView MakeBadgeView(string text)
        {
            var tv = new TextView(this) { Text = text, TextSize = 12 };
            tv.SetTextColor(Color.White);                   // High contrast foreground

            // Rounded background
            var bg = new GradientDrawable();
            bg.SetCornerRadius(Dp(8));
            bg.SetColor(Color.Gray);                        // Neutral default; updated later by SetBadge
            tv.SetPadding(Dp(8), Dp(2), Dp(8), Dp(2));      // Badge-like horizontal padding
            tv.Background = bg;
            return tv;
        }


        /// <summary>
        /// Updates the badge text and background color based on the device type.
        /// Reuses the existing rounded <see cref="GradientDrawable"/> if present.
        /// </summary>
        /// <param name="tv">Badge TextView to update.</param>
        /// <param name="type">Detected device type (IMU/EXG/DeviceOff/Unknown).</param>
        void SetBadge(TextView tv, ShimmerScanManager.DeviceType type)
        {

            // Map type to (label, color)
            var (txt, col) = type switch
            {
                ShimmerScanManager.DeviceType.IMU => ("IMU", Color.ParseColor("#2E7D32")),
                ShimmerScanManager.DeviceType.EXG => ("EXG", Color.ParseColor("#1565C0")),
                ShimmerScanManager.DeviceType.DeviceOff => ("OFF", Color.ParseColor("#757575")),
                _ => ("?", Color.ParseColor("#EF6C00")),
            };
            tv.Text = txt;

            // If the badge already has a rounded drawable, recolor it; otherwise create one
            if (tv.Background is GradientDrawable gd) gd.SetColor(col);
            else
            {
                var bg = new GradientDrawable();
                bg.SetCornerRadius(Dp(8));
                bg.SetColor(col);
                tv.Background = bg;
            }
        }


        /// <summary>
        /// Connects to all selected devices, validates sensor choices, builds per-device configs,
        /// starts the WebSocket bridge, and sequentially opens/configures/starts each device.
        /// Shows a progress dialog and a final summary with success/errors.
        /// </summary>
        async Task ConnectAndStartAsync()
        {

            // Gather devices with "Connect" checked
            var selected = _devices
                .Where(x => x.Connect.Checked)
                .ToList();

            if (selected.Count == 0)
            {
                var toast = Toast.MakeText(this!, "Please select at least one device.", ToastLength.Long);
                toast?.Show();
                _status.Text = "Status: no device selected";
                return;
            }

            // Validate: at least one sensor enabled per selected device
            var noSensor = selected.Where(x => !AnySensorEnabled(x)).ToList();
            if (noSensor.Count > 0)
            {
                var names = string.Join("\n", noSensor.Select(x => $"- {x.Name} [{x.Mac}]"));

                var builder = new Android.App.AlertDialog.Builder(this!);
                builder.SetTitle("No sensors selected");
                builder.SetMessage($"Enable at least one sensor for:\n{names}");
                builder.SetPositiveButton("OK", (s, e) => { });

                var alert = builder.Create();  
                alert?.Show();                 

                _status.Text = "Status: sensor selection required";
                return;
            }

            // Build configs from UI (EXG extras only for EXG devices)
            var targets = selected.Select(x =>
            {
                var cfg = BuildConfigFromUi(x);

                if (x.Type == ShimmerScanManager.DeviceType.EXG)
                {
                    var wire = UiModeToWire(x.SelectedExgMode);
                    cfg.ExgModeWire = wire;     // Save user’s EXG choice
                    cfg.SamplingRate = 51.2;    // Or 128.0 for higher responsiveness

                    Android.Util.Log.Info(TAG, $"[UI] {x.Name} [{x.Mac}] exg_mode (wire)='{wire}'");
                }

                return (x, Cfg: cfg);
            }).ToList();

            // Ensure WS bridge is up
            await _ws.StartAsync(this, 8787);
            _status.Text = "Status: starting selected devices…";

            // Progress dialog while connecting
            var progress = new ProgressBar(this!) { Indeterminate = true };
            var connectBuilder = new Android.App.AlertDialog.Builder(this!);
            connectBuilder.SetTitle("Connecting…");
            connectBuilder.SetView(progress);
            connectBuilder.SetCancelable(false);
            var connectDlg = connectBuilder.Create();
            connectDlg?.Show();


            var results = new List<(string name, string mac, bool ok, string? error)>();
            try
            {
                foreach (var t in targets)
                {
                    try
                    {
                        await Task.Run(async () => await _ws.OpenConfigureAndStartAsync(t.x.Mac, t.Cfg));
                        results.Add((t.x.Name, t.x.Mac, true, null));
                    }
                    catch (Exception ex)
                    {
                        results.Add((t.x.Name, t.x.Mac, false, ex.Message));
                    }
                }

                var okCount = results.Count(r => r.ok);
                var errCount = results.Count - okCount;
                _status.Text = $"Status: streaming {okCount} device(s); errors: {errCount}";
            }
            finally
            {
                try { connectDlg?.Dismiss(); } catch { }
            }

            // Show per-device summary
            var msg = string.Join("\n", results.Select(r => r.ok
                ? $"✓ {r.name} [{r.mac}] — OK"
                : $"✗ {r.name} [{r.mac}] — {r.error}"));

            var summaryBuilder = new Android.App.AlertDialog.Builder(this!);
            summaryBuilder.SetTitle("Connection result");
            summaryBuilder.SetMessage(msg);
            summaryBuilder.SetPositiveButton("OK", (s, e) => { });
            var summaryDlg = summaryBuilder.Create();
            summaryDlg?.Show();
        }


        /// <summary>
        /// Stops all active device streams via the WebSocket bridge.
        /// Shows a non-cancelable progress dialog while shutting down, then updates the status to Idle
        /// and displays a summary dialog indicating how many sessions were previously active.
        /// </summary>
        /// <returns>A task that completes after all sessions are closed and the UI is updated.</returns>
        private async Task StopAllWithNoticeAsync()
        {
            int before = _ws.ActiveSessionCount;

            // Modal progress while stopping
            var pb = new ProgressBar(this!) { Indeterminate = true };
            var builder = new Android.App.AlertDialog.Builder(this!);
            builder.SetTitle("Stopping…");
            builder.SetView(pb);
            builder.SetCancelable(false);
            var dlg = builder.Create();
            dlg?.Show();

            try
            {

                // Close every active session
                await _ws.CloseAllAsync();
            }
            finally
            {
                try { dlg?.Dismiss(); } catch { }
            }

            _status.Text = "Status: Idle";

            // User-facing summary
            var summaryBuilder = new Android.App.AlertDialog.Builder(this!);
            summaryBuilder.SetTitle("Stopped");
            summaryBuilder.SetMessage(before > 0
                ? $"Streaming stopped on {before} device(s)."
                : "No active streams.");
            summaryBuilder.SetPositiveButton("OK", (s, e) => { });
            var summaryDlg = summaryBuilder.Create();
            summaryDlg?.Show();
        }


        /// <summary>
        /// Lifecycle cleanup: disposes any pending debounce timers and stops the WebSocket bridge,
        /// which also closes all active device sessions.
        /// </summary>
        protected override async void OnDestroy()
        {
            base.OnDestroy();

            // Stop and dispose all per-MAC debounce timers (best-effort)
            foreach (STimers.Timer t in _debouncers.Values)
            {
                try { t.Stop(); t.Dispose(); } catch { }
            }
            _debouncers.Clear();

            await _ws.StopAsync();  // Gracefully stop WS server and sessions
        }


        /// <summary>
        /// Maps the user-selected EXG UI mode to the wire string expected by the server.
        /// Falls back to "ecg" if the input is null/unknown.
        /// </summary>
        /// <param name="s">UI label (e.g., "ECG", "EMG", "EXG Test", "Respiration").</param>
        /// <returns>Normalized wire token: "ecg", "emg", "test", or "resp".</returns>
        static string UiModeToWire(string s) => (s ?? "").Trim().ToLowerInvariant() switch
        {
            "ecg" => "ecg",
            "emg" => "emg",
            "exg test" => "test",
            "respiration" => "resp",
            _ => "ecg" // sicurezza: se qualcosa va storto, parti da ECG
        };


        /// <summary>
        /// Lightweight view-model for a device card: holds references to the card root,
        /// all sensor toggles, the connect checkbox, the IMU/EXG badge, and the EXG controls,
        /// plus current selection state (EXG mode and detected device type).
        /// </summary>
        sealed class DeviceUi
        {

            public View Root { get; }                 // Card root view
            public string Name { get; }               // Device display name
            public string Mac { get; }                // Device MAC

            public CheckBox Connect { get; }          // Connect toggle

            public CheckBox LnAcc { get; }            // Low-noise accelerometer
            public CheckBox WrAcc { get; }            // Wide-range accelerometer
            public CheckBox Gyro { get; }             // Gyroscope
            public CheckBox Mag { get; }              // Magnetometer
            public CheckBox Press { get; }            // Pressure & Temperature
            public CheckBox Batt { get; }             // Battery
            public CheckBox A6 { get; }               // Ext A6
            public CheckBox A7 { get; }               // Ext A7
            public CheckBox A15 { get; }              // Ext A15

            public TextView Badge { get; }            // Type badge (IMU/EXG/?/OFF)
            public LinearLayout ExgRow { get; }       // EXG controls row
            public RadioGroup ExgGroup { get; }       // EXG mode selector
            public string SelectedExgMode { get; set; } = "ECG"; // Current EXG mode


            // Detected type
            public ShimmerScanManager.DeviceType Type { get; set; } = ShimmerScanManager.DeviceType.Unknown;


            /// <summary>
            /// Constructor wiring all UI controls and identifiers for a device card.
            /// </summary>
            /// <param name="root">Card root view.</param>
            /// <param name="name">Device display name.</param>
            /// <param name="mac">Device MAC address.</param>
            /// <param name="connect">Connect checkbox.</param>
            /// <param name="ln">Low-noise accelerometer checkbox.</param>
            /// <param name="wr">Wide-range accelerometer checkbox.</param>
            /// <param name="gy">Gyroscope checkbox.</param>
            /// <param name="mg">Magnetometer checkbox.</param>
            /// <param name="pr">Pressure &amp; Temperature checkbox.</param>
            /// <param name="bt">Battery checkbox.</param>
            /// <param name="a6">External ADC A6 checkbox.</param>
            /// <param name="a7">External ADC A7 checkbox.</param>
            /// <param name="a15">External ADC A15 checkbox.</param>
            /// <param name="badge">Type/status badge.</param>
            /// <param name="exgRow">EXG controls row.</param>
            /// <param name="exgGroup">EXG mode RadioGroup.</param>
            public DeviceUi(View root, string name, string mac, CheckBox connect,
                CheckBox ln, CheckBox wr, CheckBox gy, CheckBox mg, CheckBox pr,
                CheckBox bt, CheckBox a6, CheckBox a7, CheckBox a15,
                TextView badge, LinearLayout exgRow, RadioGroup exgGroup)
            {
                Root = root; Name = name; Mac = mac;
                Connect = connect;
                LnAcc = ln; WrAcc = wr; Gyro = gy; Mag = mg; Press = pr; Batt = bt; A6 = a6; A7 = a7; A15 = a15;
                Badge = badge; ExgRow = exgRow; ExgGroup = exgGroup;
            }
        }
    }
}
