package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
 * Mecanum drive. TeleOp drives robot-centric: pushing the stick forward always moves
 * toward the robot's front, wherever it is facing.
 *
 * The Pedro follower (paths and odometry) only exists when the config has a Pinpoint.
 * Without one TeleOp still drives; the Pedro autos need it.
 */
public class Drivetrain implements Subsystem {

    public static double SLOW_SCALE = 0.4;
    public static double STICK_DEADZONE = 0.05;

    private final Mecanum mecanum;
    /** Null when the config has no Pinpoint. */
    public final Follower follower;

    public Drivetrain(HardwareMap hardwareMap) {
        mecanum = Constants.createDrivetrain(hardwareMap);
        follower = hardwareMap.tryGet(GoBildaPinpointDriver.class, Constants.PINPOINT_NAME) == null
                ? null
                : new Follower(Constants.createLocalizer(hardwareMap), mecanum, Constants.createAlgorithm());
    }

    /**
     * Robot-centric drive, each input -1..1.
     * forward: toward the robot's front. strafe: positive is left. turn: positive is counter-clockwise.
     */
    public void drive(double forward, double strafe, double turn, boolean slow) {
        double scale = slow ? SLOW_SCALE : 1.0;
        DrivePowers powers = new DrivePowers(
                deadzone(forward) * scale, deadzone(strafe) * scale, deadzone(turn) * scale);
        if (follower != null) {
            follower.manual(powers);
        } else {
            mecanum.drive(powers, true);
        }
    }

    public boolean hasOdometry() {
        return follower != null;
    }

    private static double deadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0.0 : value;
    }

    @Override
    public void update() {
        if (follower != null) {
            follower.update();
        }
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.stop();
        }
        mecanum.stop();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        if (follower == null) {
            telemetry.addData("Pose", "no Pinpoint in config");
            return;
        }
        Pose pose = follower.pose();
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.0f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }
}
