package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
 * Mecanum drive backed by the Pedro Pathing follower, so TeleOp and Auto share one
 * motor/localizer config (pedroPathing/Constants).
 */
public class Drivetrain implements Subsystem {

    public static double SLOW_SCALE = 0.4;

    public final Follower follower;

    private boolean fieldCentric = false;
    private double scale = 1.0;

    public Drivetrain(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
    }

    public void startTeleOp(Pose startPose) {
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();
    }

    /**
     * All inputs -1..1. Positive forward drives away from the driver,
     * positive strafe goes left, positive turn is counter-clockwise.
     */
    public void drive(double forward, double strafe, double turn) {
        follower.setTeleOpDrive(forward * scale, strafe * scale, turn * scale, !fieldCentric);
    }

    public void setSlowMode(boolean slow) {
        scale = slow ? SLOW_SCALE : 1.0;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    /** Makes the robot's current facing "forward" for field-centric driving. */
    public void resetHeading() {
        Pose pose = follower.getPose();
        follower.setPose(new Pose(pose.getX(), pose.getY(), 0));
    }

    public Pose getPose() {
        return follower.getPose();
    }

    @Override
    public void update() {
        follower.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        Pose pose = follower.getPose();
        telemetry.addData("Drive", fieldCentric ? "field-centric" : "robot-centric");
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f°",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }
}
