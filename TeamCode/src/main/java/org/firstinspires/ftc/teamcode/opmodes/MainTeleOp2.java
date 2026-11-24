package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

/*
 * MainTeleOp2 — copy of Oct24TeleOp with intake added.
 *
 * gamepad1
 *   left stick X     forward / back   (robot-centric)
 *   left stick Y     strafe left / right (robot-centric)
 *   right stick X    rotate
 *   right bumper     intake forward (hold)  — Expansion Hub port 0
 *   left bumper      intake reverse (hold)  — Expansion Hub port 0
 */
@TeleOp(name = "MainTeleOp2", group = "BIOBUZZ")
public class MainTeleOp2 extends LinearOpMode {

    private static final double DEADZONE     = 0.10;
    private static final double INTAKE_POWER = 0.5;

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
        DcMotor intake     = hardwareMap.get(DcMotor.class, "intake");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor[] driveMotors = {leftFront, leftRear, rightFront, rightRear};
        for (DcMotor motor : driveMotors) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intake.setPower(0.0);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("MainTeleOp2 ready");
        telemetry.addLine("Left stick: move  |  Right stick: spin  |  RB: intake fwd  |  LB: intake rev");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ── Drive ─────────────────────────────────────────────────────────
            double f = deadzone(-gamepad1.left_stick_x);
            double s = deadzone(-gamepad1.left_stick_y);
            double t = deadzone(gamepad1.right_stick_x);

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

            // ── Intake ───────────────────────────────────────────────────────
            // RB = forward, LB = reverse, neither = off
            if (gamepad1.right_bumper) {
                intake.setPower(INTAKE_POWER);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-INTAKE_POWER);
            } else {
                intake.setPower(0.0);
            }

            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            // ── Telemetry ─────────────────────────────────────────────────────
            telemetry.addData("Sticks", "f %.2f  s %.2f  t %.2f", f, s, t);
            telemetry.addData("Wheels", "LF %.2f  LR %.2f  RF %.2f  RR %.2f",
                    leftFront.getPower(), leftRear.getPower(),
                    rightFront.getPower(), rightRear.getPower());
            telemetry.addData("Intake", gamepad1.right_bumper ? "FWD" : gamepad1.left_bumper ? "REV" : "off");
            telemetry.update();
        }

        for (DcMotor motor : driveMotors) {
            motor.setPower(0.0);
        }
        intake.setPower(0.0);
    }

    /** Kills tiny joystick drift around the center. */
    private static double deadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0.0 : value;
    }
}
