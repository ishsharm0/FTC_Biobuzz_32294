# Team 32294 AUTO assumptions and checks

This route matches the front intake and rear outtake described by the team on 2026-11-10, using0.9s between loaded shots.
Park Auto leaves the alliance wall and parks. Launch Park Auto begins with its front touching the audience wall,
backs toward the launch stand, sends three separate transfer pulses, waits for hive settling, and curves around the frame to park.
The fourth preload is retained because the initial three NECTAR plus three POLLEN can tip the raised cell.

Official sources: [G304/G402 and transition rule G403](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-11),
[staging and scoring, sections 10.3 and 10.5](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-10).
Wall, loading-tape and hive offsets are converted from the simulator's official field CAD manifest/config.

The source starts at the wall; the simulator places it there before AUTO. There is no extra drive from a default spawn.
The 18 in. robot dimensions are still assumptions. The simulator's 20 in. nominal launch stand and reference ballistics are
not a physical tune. RearLauncher uses optional `transfer` hardware, direction REVERSE; no independent flywheel is mapped.
Its 0.25 s feeder pulse needs measurement; without a ball sensor one pulse is only an estimate of one ball.
The 0.9 s cycle is a team-supplied estimate. Missing hardware results in drive/park only. A bad launch pose aborts firing.
Set Pedro/Pinpoint constants, measure dimensions and the launch stand, and validate feed timing on the real field.

Checks: JVM tests cover pulse/recovery/cancel timing, wall start contact, rear facing, blue symmetry and 1001 full-footprint samples per path.
Simulator fixtures are exact copies of Field.java and the four match path/opmode files. Its `scripts/team-auto.ts` exercises both
alliances over three seeds; `--write-plan` regenerates built-in 32294-custom. These checks do not constitute a real-robot test.
