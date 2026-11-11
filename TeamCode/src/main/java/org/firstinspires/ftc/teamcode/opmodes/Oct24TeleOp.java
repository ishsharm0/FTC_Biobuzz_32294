package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * gamepad1
 *   left stick      drive / strafe
 *   right stick X   turn
 *   left bumper     slow mode (hold)
 *   right bumper    slow mode (hold)
 *   right trigger   intake (hold)
 *   left trigger     transfer (hold)
 */
@TeleOp(name = "Oct24 TeleOp", group = "BIOBUZZ")
public class Oct24TeleOp extends LinearOpMode {

    public static double SLOW_SCALE     = 0.4;
    public static double INTAKE_POWER   = 0.3;
    public static double TRANSFER_POWER = 1.0;

    @Override
    public void runOpMode() {
        DcMotor leftFront  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor leftRear   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor rightRear  = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor intake     = hardwareMap.get(DcMotor.class, "intake");
        DcMotor transfer   = hardwareMap.get(DcMotor.class, "transfer");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] motors = {leftFront, leftRear, rightFront, rightRear, intake, transfer};
        for (DcMotor motor : motors) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Oct24 TeleOp ready");
        telemetry.addLine("LB/RB: slow  |  RT: intake  |  LT: transfer");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ── Drivetrain ────────────────────────────────────────────────────
            double f = -gamepad1.left_stick_y;
            double s = -gamepad1.left_stick_x;
            double t = -gamepad1.right_stick_x;

            double lf = f - s - t;
            double lr = f + s - t;
            double rf = f + s + t;
            double rr = f - s + t;

            double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lr)),
                    Math.max(Math.abs(rf), Math.abs(rr))));
            boolean slowMode = gamepad1.left_bumper || gamepad1.right_bumper;
            double scale = (slowMode ? SLOW_SCALE : 1.0) / max;

            leftFront.setPower(lf * scale);
            leftRear.setPower(lr * scale);
            rightFront.setPower(rf * scale);
            rightRear.setPower(rr * scale);

            // ── Mechanisms ────────────────────────────────────────────────────
            intake.setPower(gamepad1.right_trigger > 0.1 ? INTAKE_POWER : (gamepad1.left_trigger > 0.1 ? -INTAKE_POWER : 0.0));
//            transfer.setPower(gamepad1.left_trigger > 0.1 ? TRANSFER_POWER : 0.0);

            // ── Telemetry ─────────────────────────────────────────────────────
            telemetry.addData("Mode",     slowMode ? "slow" : "normal");
            telemetry.addData("Front L/R", "%.2f  %.2f", leftFront.getPower(), rightFront.getPower());
            telemetry.addData("Back  L/R", "%.2f  %.2f", leftRear.getPower(), rightRear.getPower());
            telemetry.addData("Intake",   gamepad1.right_trigger > 0.1 ? "ON" : "off");
            telemetry.addData("Transfer", gamepad1.left_trigger  > 0.1 ? "ON" : "off");
            telemetry.update();
        }

        for (DcMotor motor : motors) {
            motor.setPower(0.0);
        }
    }
}
