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
| `intake`, `transfer` | Front intake and rear feed/outtake. Match auto pulses `transfer`; optional for Park Auto |
| `Webcam 1` | USB camera, optional. Main TeleOp runs without it |

## 5. Drive

Run **Basic TeleOp** first (group BIOBUZZ). It uses only the four motors: left stick drives/strafes,
right stick turns, hold right bumper for slow mode. If a wheel spins the wrong way, flip its direction
in `opmodes/BasicTeleOp.java`.

**Main TeleOp** and **Auto Template** use Pedro Pathing + the Pinpoint. Don't trust paths until Pedro is tuned:
open `http://192.168.43.1:10158` while connected to the hub and run Mecanum → Pinpoint → Foresight → Tests,
pasting each result into `pedroPathing/Constants.java`.

## 6. Match auto

**Park Auto** / **Launch Park Auto** (Pedro + Pinpoint, so tune first). After INIT, on gamepad1: X / B picks
blue / red, dpad up/down sets a start delay. Telemetry says where to place the robot.

| OpMode | Start | Does | Points |
| --- | --- | --- | --- |
| Park Auto | back on the alliance wall, just audience-side of the loading zone | slides into the loading zone | LEAVE 3 + PARK 5 |
| Launch Park Auto | front on the audience wall, rear outtake facing the raised cell | three single preload feed pulses, settle, then park | LEAVE3 + PARK5; successful HIVE TIP adds20 |

Both robots leaving and parking is 16 points, which is the SWARM ranking point.
Field positions live in `Field.java`, the paths in `opmodes/AutoPaths.java`. Set `ROBOT_LENGTH` / `ROBOT_WIDTH`
in `Field.java` to the real robot. **Oct24 Auto** and **Leave** are the timed fallbacks that need no odometry.

The robot uses the existing `transfer` motor as the rear outtake (REVERSE, as in Oct24 TeleOp).
`RearLauncher.FEED_SECONDS = 0.25` is an initial **unmeasured feed pulse**, not a ball sensor: adjust it on the robot
until one pulse ejects exactly one ball. `RearLauncher.SHOT_INTERVAL_SECONDS` and
`LaunchParkAuto.SHOT_INTERVAL_SECONDS` are both 0.9 s, the team's loaded-shot estimate; keep both values together.
No separate flywheel motor is configured by this code. A missing `transfer` skips launching and still parks.
The fourth preload stays aboard: the initial raised cell has three NECTAR, and three POLLEN can tip it.
The auto verifies its launch pose before firing, waits for the hive to settle, and stops all outputs by 29 s.
The maximum optional start delay is 10 s so time remains for parking.

Poses use Pedro's nominal 144 in. corner frame. `Field.WALL_INSET` accounts for the physical CAD wall being 1.7929 m
from center, so starts actually touch it. The rear shooter faces the hive at heading -90 deg for red; blue is rotated 180 deg.
The 20 in. launch Y is a simulator reference stand, not a measured ballistic calibration for this robot.

To try an auto in the BIOBUZZ simulator, import its file together with `AutoPaths.java` and `Field.java`.
The simulator starts at the declared initial pose, with a front intake and a rear FIFO shooter firing one ball
at a time 0.9 s apart. `scripts/team-auto.ts --write-plan` in the simulator regenerates its built-in 32294 path
from the synchronized Java fixture snapshots.

## Stuck?

- **"Cannot resolve symbol"** → File → Sync Project with Gradle Files (then File → Invalidate Caches if needed).
- **"source value 8 is obsolete" warning** → harmless, ignore.
- More help: [FTC Docs](https://ftc-docs.firstinspires.org), [Game Manual 0](https://gm0.org), [Pedro Pathing](https://pedropathing.com).
