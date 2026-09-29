package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

import static com.pedropathing.api.Paths.line;

/*
 * Starting point for autos: a state machine that runs one step per loop.
 * Each step starts a path (or a mechanism action) and moves on once it's done.
 * Poses are in inches/radians on the Pedro field (0..144).
 */
@Autonomous(name = "Auto Template", group = "BIOBUZZ")
public class AutoTemplate extends OpMode {

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose parkPose = new Pose(56, 36, Math.toRadians(90));

    private Robot robot;
    private Follower follower;
    private Path park;
    private int step;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        follower = robot.drivetrain.follower;
        follower.setPose(startPose);

        park = line(startPose, parkPose).linear(startPose, parkPose);
    }

    @Override
    public void start() {
        step = 0;
    }

    @Override
    public void loop() {
        robot.update();

        switch (step) {
            case 0:
                follower.follow(park);
                step++;
                break;
            case 1:
                if (!follower.isBusy()) {
                    step++;
                }
                break;
            default:
                // Done. Holding the last pose.
                break;
        }

        telemetry.addData("Step", step);
        robot.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
