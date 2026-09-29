package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static com.pedropathing.api.Paths.line;

import java.util.Arrays;
import java.util.List;

/*
 * Mecanum drive. Two ways to drive it in TeleOp:
 *
 *   Mode.PEDRO     the follower drives the motors. Field-centric comes free and the pose
 *                  stays accurate, so you can hand off to a path mid-match.
 *   Mode.VELOCITY  we drive the motors ourselves with setVelocity, so a stick position means
 *                  the same wheel speed whether the battery is full or nearly flat.
 *                  Sticks get expo (finer control near center) and slew (no instant jumps).
 *
 * Either way odometry keeps updating, so the pose stays live and driveTo() works.
 * Auto just uses the follower and never touches VELOCITY.
 */
public class Drivetrain implements Subsystem {

    public enum Mode { PEDRO, VELOCITY }

    public static double SLOW_SCALE = 0.4;

    // goBILDA 5203 312 RPM. Change these if the drive motors change.
    public static double TICKS_PER_REV = 537.7;
    public static double MAX_RPM = 312.0;
    public static double VELOCITY_HEADROOM = 0.85;

    public static double DRIVE_EXPO = 0.45;
    public static double TURN_EXPO = 0.35;
    public static double DRIVE_SLEW_PER_SEC = 4.0;
    public static double TURN_SLEW_PER_SEC = 7.0;

    public final Follower follower;

    private final DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private final List<DcMotorEx> motors;

    private Mode mode = Mode.PEDRO;
    private boolean fieldCentric = false;
    private double scale = 1.0;

    private long lastCommandNs = 0L;
    private double lastForward = 0.0, lastStrafe = 0.0, lastTurn = 0.0;

    public Drivetrain(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);

        MecanumConfig config = Constants.drivetrainConfig;
        leftFront = hardwareMap.get(DcMotorEx.class, config.frontLeftName.get());
        leftRear = hardwareMap.get(DcMotorEx.class, config.backLeftName.get());
        rightFront = hardwareMap.get(DcMotorEx.class, config.frontRightName.get());
        rightRear = hardwareMap.get(DcMotorEx.class, config.backRightName.get());
        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        leftFront.setDirection(config.frontLeftDirection.get());
        leftRear.setDirection(config.backLeftDirection.get());
        rightFront.setDirection(config.frontRightDirection.get());
        rightRear.setDirection(config.backRightDirection.get());
    }

    public void startTeleOp(Pose startPose) {
        follower.setPose(startPose);
    }

    /**
     * All inputs -1..1. Positive forward drives away from the driver,
     * positive strafe goes left, positive turn is counter-clockwise.
     */
    public void drive(double forward, double strafe, double turn) {
        if (mode == Mode.PEDRO) {
            DrivePowers powers = new DrivePowers(forward * scale, strafe * scale, turn * scale);
            follower.manual(fieldCentric ? ManualDrive.fieldCentric(powers, follower.pose().heading()) : powers);
            return;
        }

        long now = System.nanoTime();
        double dt = lastCommandNs == 0L ? 0.02 : Range.clip((now - lastCommandNs) / 1e9, 0.005, 0.05);
        lastCommandNs = now;

        double f = slew(expo(forward, DRIVE_EXPO), lastForward, DRIVE_SLEW_PER_SEC * dt);
        double s = slew(expo(strafe, DRIVE_EXPO), lastStrafe, DRIVE_SLEW_PER_SEC * dt);
        double t = slew(expo(turn, TURN_EXPO), lastTurn, TURN_SLEW_PER_SEC * dt);
        lastForward = f;
        lastStrafe = s;
        lastTurn = t;

        if (fieldCentric) {
            double heading = follower.pose().heading();
            double cos = Math.cos(-heading), sin = Math.sin(-heading);
            double rotatedF = f * cos - s * sin;
            s = f * sin + s * cos;
            f = rotatedF;
        }

        double lf = f - s - t;
        double lr = f + s - t;
        double rf = f + s + t;
        double rr = f - s + t;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lr)),
                Math.max(Math.abs(rf), Math.abs(rr))));
        double ticksPerSec = scale * maxTicksPerSec() / max;

        leftFront.setVelocity(lf * ticksPerSec);
        leftRear.setVelocity(lr * ticksPerSec);
        rightFront.setVelocity(rf * ticksPerSec);
        rightRear.setVelocity(rr * ticksPerSec);
    }

    /** Drives a straight line to a field pose using odometry. Auto, or a TeleOp macro. */
    public void driveTo(Pose target) {
        setMode(Mode.PEDRO);
        Pose current = follower.pose();
        follower.follow(line(current, target).linear(current, target));
    }

    /** True while a driveTo() or path is still running. */
    public boolean isBusy() {
        return follower.isBusy();
    }

    public Mode getMode() {
        return mode;
    }

    public void setMode(Mode mode) {
        if (this.mode == mode) {
            return;
        }
        this.mode = mode;

        if (mode == Mode.VELOCITY) {
            // Stops the follower writing motor powers, then take the motors over.
            // update() only runs the localizer in this mode so the follower stays off them.
            follower.stop();
            follower.drivetrain.stop();
            for (DcMotorEx motor : motors) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            lastCommandNs = 0L;
            lastForward = lastStrafe = lastTurn = 0.0;
        } else {
            for (DcMotorEx motor : motors) {
                motor.setVelocity(0.0);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
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
        follower.setHeading(0);
    }

    public Pose getPose() {
        return follower.pose();
    }

    private static double maxTicksPerSec() {
        return VELOCITY_HEADROOM * (MAX_RPM / 60.0) * TICKS_PER_REV;
    }

    private static double expo(double input, double amount) {
        double magnitude = Math.abs(input);
        return Math.copySign((1.0 - amount) * magnitude + amount * magnitude * magnitude * magnitude, input);
    }

    private static double slew(double target, double last, double maxStep) {
        return last + Range.clip(target - last, -maxStep, maxStep);
    }

    @Override
    public void update() {
        if (mode == Mode.VELOCITY) {
            follower.localizer.update();
        } else {
            follower.update();
        }
    }

    @Override
    public void stop() {
        if (mode == Mode.VELOCITY) {
            for (DcMotorEx motor : motors) {
                motor.setVelocity(0.0);
            }
        }
        follower.stop();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        Pose pose = follower.pose();
        telemetry.addData("Drive", "%s, %s", mode, fieldCentric ? "field-centric" : "robot-centric");
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f°",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }
}
