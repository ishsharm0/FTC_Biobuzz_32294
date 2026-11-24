# BIOBUZZ 32294 Quick Start

## 1. Install

- [Android Studio](https://developer.android.com/studio). Let it install the Android SDK on first launch.
  It also brings the Java the build uses; no separate JDK needed.
- [Git](https://git-scm.com/downloads) (macOS: `xcode-select --install`).

Clone `https://github.com/ishsharm0/FTC_Biobuzz_32294.git` (work goes on `main`), open it in Android Studio and let Gradle sync.
**Build → Make Project** should end in `BUILD SUCCESSFUL`. If Android Studio offers to upgrade Gradle
or the Android Gradle plugin, say no.

From a terminal: `./gradlew :TeamCode:assembleDebug` (set `JAVA_HOME` to Android Studio's
`Contents/jbr/Contents/Home` on macOS if your system Java is older than 17).

## 2. Deploy

**macOS, no Android Studio needed:** `./deploy` from the repo root. It builds on your normal Wi-Fi,
switches to the robot's Wi-Fi (`32294-NEW`), pushes, and switches back.

| Command | Does |
| --- | --- |
| `./deploy` | Sloth hot-reload of TeamCode in a few seconds; a full install when anything outside TeamCode's Java changed |
| `./deploy full` | full APK install, restarts the Robot Controller app |
| `./deploy log` | robot logcat until Ctrl-C |
| `./deploy status` | Wi-Fi and adb state |

One-time: `./deploy setup` (asks for your Mac password once, so the script can see which Wi-Fi to return to).
If it can't join the robot Wi-Fi on its own, put `ROBOT_PASS=...` in `.deploy.local` (gitignored).
A Control Hub plugged in by USB is used directly, without switching Wi-Fi.

**Android Studio:** plug in the hub by USB, or join its Wi-Fi and run `adb connect 192.168.43.1:5555`,
then Run ▶ with the **TeamCode** configuration.

## 3. Robot configuration

Names in the Driver Station's active config must match exactly:

| Name | Device | Needed by |
| --- | --- | --- |
| `frontLeft`, `backLeft`, `frontRight`, `backRight` | drive motors | everything |
| `intake` | front intake motor | optional |
| `transfer` | rear outtake feed motor | optional |
| `pinpoint` | goBILDA Pinpoint (I2C) | Park Auto, Launch Park Auto |
| `Webcam 1` | USB camera | optional |

## 4. OpModes

| OpMode | What it is |
| --- | --- |
| **Main TeleOp** | robot-centric drive. Left stick drive/strafe, right stick turn, LT slow, RB intake in, LB intake out, A feed launcher, Y show HIVE target |
| **Park Auto** | Pedro: back on the alliance wall beside the loading zone, slides into it. LEAVE 3 + PARK 5 |
| **Launch Park Auto** | Pedro: front on the audience wall, three single shots into the raised CELL, then park |
| **Timed Leave** | no odometry: drives forward for `DRIVE_SECONDS`. Use when the Pinpoint isn't working |
| **DEBUG** | every motor and servo one at a time; DRIVE CHECK page verifies the drive motor directions |
| **AprilTag Demo** | live AprilTag detection: distance and angles to every tag, camera tuning, hold LB to turn and face the closest tag. Watch it on the Driver Station: during INIT, menu → Camera Stream |

In the Pedro autos, during INIT on gamepad1: X / B picks blue / red, dpad up/down sets a start delay.
Telemetry says where to place the robot.

Drive motor names and directions live in one place, `pedroPathing/Constants.java`. If a wheel turns the
wrong way, confirm it on DEBUG → DRIVE CHECK (hold X: every wheel should roll the robot forward) and change it there.

## 5. Tuning

- **Pedro:** paths aren't trustworthy until tuned. Deploy, join the robot Wi-Fi, open
  `http://192.168.43.1:10158`, run Mecanum → Pinpoint → Foresight → Tests, and paste each result into
  `pedroPathing/Constants.java`.
- **Launcher:** `RearLauncher.FEED_SECONDS` is a guess. Adjust until one pulse fires exactly one ball.
- **Robot size:** set `Field.ROBOT_LENGTH` / `ROBOT_WIDTH` to the real bumper-to-bumper size; the auto paths use them.
- Field positions are in `Field.java`, the paths in `opmodes/AutoPaths.java`. Run `./gradlew :TeamCode:testDebugUnitTest`
  after changing them: the tests check the robot stays clear of the walls and the HIVE frame.

## Stuck?

- **"Cannot resolve symbol"** → File → Sync Project with Gradle Files (then File → Invalidate Caches).
- [FTC Docs](https://ftc-docs.firstinspires.org), [Game Manual 0](https://gm0.org), [Pedro Pathing](https://pedropathing.com).
