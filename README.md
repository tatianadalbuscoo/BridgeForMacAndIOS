# BridgeForMacAndIOS

A .NET/C# **bridge** that enables the **same .NET MAUI app** to run on **iOS** and **MacCatalyst** by providing platform glue code and project wiring.  
It is designed to be used together with the MAUI app here: **ShimmerMobileApp** -> https://github.com/tatianadalbuscoo/ShimmerMobileApp

> In short: this repo exists so you can ship one MAUI app that runs on Windows, Android, iOS, and Mac (via MacCatalyst), reusing the same UI and business logic.

---

## Overview

**Important (iOS/MacCatalyst):** Shimmer3 uses **Bluetooth Classic (SPP/RFCOMM via RN-42)**. Apps on iOS and Mac **via MacCatalyst** can only use **CoreBluetooth (BLE/GATT)**; SPP is available **only** to MFi/iAP accessories.  
**Consequence:** you cannot pair directly to Shimmer3 from iOS or MacCatalyst.

**Why Mac can’t do SPP in this app:** On Mac, the MAUI app runs as **MacCatalyst**, which reuses the **iOS API surface**. That means you still get **CoreBluetooth-only (BLE/GATT)** and **no RFCOMM/SPP**, even though macOS natively supports RFCOMM in other frameworks. MacCatalyst does **not** expose those macOS-only APIs.

**What this bridge does:**  
This repository provides an **Android-based bridge** so you can keep your **Shimmer3 (Bluetooth Classic / SPP)** hardware and still use the **same .NET MAUI app** on **iOS** and **Mac (via MacCatalyst)**.

**How it works (no hardware change):**
- An **Android device** connects to **Shimmer3** over **Bluetooth Classic (SPP/RFCOMM)**.
- The Android bridge **reads the sensor stream** and **re-publishes** it on the local network (**WebSocket**).
- The **.NET MAUI app** running on **iPhone/iPad (iOS)** or **Mac (MacCatalyst)** then **connects to the Android device’s IP:port** on the same Wi‑Fi/hotspot and **receives the packets as-is**.

This repository also includes a small test suite to prevent regressions.

---

## Prerequisites (for this bridge only)

- **Android device (bridge)**: phone/tablet with **Bluetooth Classic (SPP/RFCOMM)** and Wi-Fi.
- **.NET SDK 8.0** to build the Android app.
  - MAUI-based bridge: `dotnet workload install maui`
  - Plain Android project: `dotnet workload install android`
- **Git** (to clone this repo).
- **Network**: the **Android bridge** and the **MAUI client (iOS/MacCatalyst)** must be on the **same Wi-Fi/hotspot**.  
  You’ll need the Android device **IP** and the open **port** (default: `8787` for WS).
- **Android permissions**: Bluetooth (Classic), Bluetooth Admin/Connect/Scan (API 31+), and **INTERNET**.
  
---

## Getting Started

Clone & restore:

```bash
git clone https://github.com/tatianadalbuscoo/BridgeForMacAndIOS
cd BridgeForMacAndIOS
dotnet restore
```

You can proceed in **two way** to deploy the Android bridge:

- **Visual Studio**:  
  Open `BridgeForMacAndIOS.sln` -> set the **ShimmerAndroidBridge** project as *Startup Project* -> select your **Android device** -> **Debug -> Start Debugging**.

- **Visual Studio Code** (with **C# Dev Kit** + Android tooling):  
  Open the folder -> choose the **ShimmerAndroidBridge** folder -> In the visual studio code terminal run:
  ```bash
  dotnet build .\ShimmerAndroidBridge\ShimmerAndroidBridge.csproj -f net8.0-android -c Debug -t:Run
  ```

**Important**: Make sure your Android phone/tablet is in **Developer mode** with **USB debugging** enabled.

---

## Run Tests

Use the provided command to execute the test suite:
```bash
dotnet test tests/tests.csproj -c Debug
```
