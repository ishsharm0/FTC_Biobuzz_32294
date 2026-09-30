# BIOBUZZ 32294 Quick Start

From a fresh Android Studio install to driving the robot.

## 1. Install

- [Android Studio](https://developer.android.com/studio) (latest stable). Let it install the Android SDK on first launch.
- [Git](https://git-scm.com/downloads) (macOS: `xcode-select --install`).
- Windows only: [REV Hardware Client](https://docs.revrobotics.com/rev-hardware-client), needed for deploying over Wi-Fi.

No separate Java install needed: this project builds with the JDK bundled in Android Studio.
(Older FTC docs say to install JDK 17. That applied to older SDK versions, not SDK 12.)

## 2. Get the code

1. Android Studio → **Clone Repository** → `https://github.com/ishsharm0/FTC_Biobuzz_32294.git`
   (ask Ishaan for repo access).
2. Wait for **Gradle sync** to finish. The first one takes a few minutes.
3. Switch to your branch: branch name in the corner → `origin/<your name>` → **Checkout**.
4. **Build → Make Project** should end in `BUILD SUCCESSFUL`.

If Android Studio offers to upgrade Gradle, the Gradle plugin, or the Java version: **say no**.

## 3. Deploy to the robot

**USB:** plug in the Control Hub → pick **REV Robotics Control Hub** in the device dropdown → Run ▶ (config: **TeamCode**).

**Wi-Fi (Windows):** join the Control Hub's Wi-Fi → open REV Hardware Client (it connects ADB for you) →
the hub appears in the device dropdown → Run ▶.
([REV guide](https://docs.revrobotics.com/duo-control/managing-the-control-system/android-studio-using-wireless-adb))

On macOS/Linux, join the hub's Wi-Fi and run `adb connect 192.168.43.1:5555` in Android Studio's Terminal instead.

## 4. Robot configuration

Names in the Driver Station's active config must match exactly:

| Name | Device |
| --- | --- |
| `frontLeft`, `backLeft`, `frontRight`, `backRight` | Drive motors |
| `pinpoint` | goBILDA Pinpoint (I2C) |

## 5. Drive

Run **Basic TeleOp** first (group BIOBUZZ). It uses only the four motors: left stick drives/strafes,
right stick turns, hold right bumper for slow mode. If a wheel spins the wrong way, flip its direction
in `opmodes/BasicTeleOp.java`.

**Main TeleOp** and **Auto Template** use Pedro Pathing + the Pinpoint. Don't trust paths until Pedro is tuned:
open `http://192.168.43.1:10158` while connected to the hub and run Mecanum → Pinpoint → Foresight → Tests,
pasting each result into `pedroPathing/Constants.java`.

## Stuck?

- **"Cannot resolve symbol"** → File → Sync Project with Gradle Files (then File → Invalidate Caches if needed).
- **"source value 8 is obsolete" warning** → harmless, ignore.
- More help: [FTC Docs](https://ftc-docs.firstinspires.org), [Game Manual 0](https://gm0.org), [Pedro Pathing](https://pedropathing.com).
