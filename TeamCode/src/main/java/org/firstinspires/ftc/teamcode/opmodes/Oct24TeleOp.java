package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

/*
 * Oct24 TeleOp — joystick-only, robot-centric mecanum drive.
 *
 * gamepad1
 *   left stick X     forward / back   (relative to the robot)  -- unchanged
 *   left stick Y     strafe left / right (relative to the robot) -- unchanged
 *   right stick X    rotate: left-side wheels vs right-side wheels, always opposing
 *
 * The config names of the motors do not match their physical corners, so the
 * usual mecanum turn is applied to the front/back pairs. The turn coefficients
 * below are assigned per motor so the physical LEFT side and RIGHT side oppose
 * each other, giving a clean spin about the robot's center.
 */
@TeleOp(name = "Oct24 TeleOp", group = "BIOBUZZ")
public class Oct24TeleOp extends LinearOpMode {

    private static final double DEADZONE = 0.10;

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        DcMotor leftFront  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor leftRear   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor rightRear  = hardwareMap.get(DcMotor.class, "backRight");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor[] motors = {leftFront, leftRear, rightFront, rightRear};
        for (DcMotor motor : motors) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Oct24 TeleOp ready");
        telemetry.addLine("Left stick: move  |  Right stick: spin left/right sides");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ── Joysticks ─────────────────────────────────────────────────────
            double f = deadzone(-gamepad1.left_stick_x);   // + = forward  (LOCKED)
            double s = deadzone(-gamepad1.left_stick_y);   // + = left     (LOCKED)
            double t = deadzone(gamepad1.right_stick_x);   // + = spin right (clockwise)

            // Translation coefficients are the same as before. Only the turn
            // term is assigned front-pair vs back-pair so that, physically,
            // the left side and right side always oppose each other.
            double lf = f - s - t;
            double rf = f + s - t;
            double lr = f + s + t;
            double rr = f - s + t;

            double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lr)),
                    Math.max(Math.abs(rf), Math.abs(rr))));

            leftFront.setPower(lf / max);
            leftRear.setPower(lr / max);
            rightFront.setPower(rf / max);
            rightRear.setPower(rr / max);

            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            // ── Telemetry ─────────────────────────────────────────────────────
            telemetry.addData("Sticks", "f %.2f  s %.2f  t %.2f", f, s, t);
            telemetry.addData("Wheels", "LF %.2f  LR %.2f  RF %.2f  RR %.2f",
                    leftFront.getPower(), leftRear.getPower(),
                    rightFront.getPower(), rightRear.getPower());
            telemetry.update();
        }

        for (DcMotor motor : motors) {
            motor.setPower(0.0);
        }
    }

    /** Kills tiny joystick drift around the center. */
    private static double deadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0.0 : value;
    }
}
