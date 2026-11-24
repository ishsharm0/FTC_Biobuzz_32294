---
name: robot-deploy
description: Push code to the FTC robot (Control Hub) without Android Studio. Use when asked to deploy, push, install, hot-reload, or read logcat from the robot.
---

Run `./deploy` from the repo root as ONE Bash call with a 600000 ms timeout. Never split it into
steps: it switches this Mac to the robot's Wi-Fi (no internet) and only switches back when it exits,
so nothing that needs the network can run in between.

- `./deploy` — Sloth hot-reload of TeamCode (seconds). Switches to a full APK install automatically
  when FtcRobotController, gradle files, libs, res or the manifest changed, or when Sloth times out.
- `./deploy full` — force full install + restart the Robot Controller app.
- `./deploy log` — logcat; interactive, so tell the user to run `! ./deploy log` themselves.
- `./deploy status` / `./deploy back` — check Wi-Fi/adb; return to the previous Wi-Fi if stranded.

It builds first (on the normal Wi-Fi), skips the Wi-Fi hop if the robot is on USB, and the log of the
last run is `build/.deploy/last.log`. Settings (ROBOT_SSID, ROBOT_PASS, HOME_SSID, HOME_PASS) go in
`.deploy.local` (gitignored). If it says the previous Wi-Fi is unknown, the user needs to run
`! ./deploy setup` once (sudo, makes macOS reveal the Wi-Fi name).
