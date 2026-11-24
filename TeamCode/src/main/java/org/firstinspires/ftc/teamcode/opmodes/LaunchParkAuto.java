package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * Drive to the launch spot, fire three preloads one at a time into the raised audience
 * CELL, wait for the HIVE to settle, then park. The fourth preload stays aboard: the CELL
 * starts with three NECTAR, so three POLLEN can tip it and a fourth would land in the
 * lowered CELL. Shooting is skipped if the robot didn't reach the launch pose or has no
 * transfer motor; it still parks.
 */
@Autonomous(name = "Launch Park Auto", group = "BIOBUZZ", preselectTeleOp = "Main TeleOp")
public class LaunchParkAuto extends BiobuzzAuto {

    public static int SHOTS = 3;
    // Counted from the last shot: the pulse, then time for the HIVE to stop moving.
    public static double HIVE_SETTLE_SECONDS = 1.9;

    private int shotsFired;

    @Override
    protected Pose startPose() {
        return paths.audienceStart;
    }

    @Override
    protected String placement() {
        return "FRONT on the audience wall, REAR outtake toward the raised audience CELL";
    }

    @Override
    protected void run() {
        switch (step) {
            case 0:
                follower.follow(paths.startToLaunch);
                next();
                break;
            case 1:
                if (pathDone()) {
                    follower.hold(paths.launch);
                    // Skip straight to parking if there's nothing to shoot with or we're off target.
                    step = robot.launcher.available() && atPose(paths.launch) ? 2 : 3;
                    stepTimer.reset();
                }
                break;
            case 2:
                if (shotsFired < SHOTS) {
                    if (robot.launcher.shootOne()) {
                        shotsFired++;
                    }
                } else {
                    next();
                }
                break;
            case 3:
                if (stepTimer.seconds() >= HIVE_SETTLE_SECONDS) {
                    robot.launcher.stop();
                    follower.follow(paths.launchToPark);
                    next();
                }
                break;
            default:
                break;
        }
    }
}
