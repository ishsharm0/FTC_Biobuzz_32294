package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

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
    private PathChain park;
    private int step;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        follower = robot.drivetrain.follower;
        follower.setStartingPose(startPose);

        park = follower.pathBuilder()
                .addPath(new BezierLine(startPose, parkPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
                .build();
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
                follower.followPath(park, true);
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
