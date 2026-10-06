package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * gamepad1
 *   left stick     drive / strafe
 *   right stick X  turn
 *   right bumper   slow mode (hold)
 * Motor names and directions must match the robot configuration. If the robot
 * spins or drifts when driving straight, flip the offending motor's direction below.
 */
@TeleOp(name = "Basic TeleOp", group = "BIOBUZZ")
public class BasicTeleOp extends LinearOpMode {

    public static double SLOW_SCALE = 0.4;

    @Override
    public void runOpMode() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "backRight");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Stick Y is negative when pushed forward. Positive strafe is left, positive turn is CCW.
            double f = -gamepad1.left_stick_y;
            double s = -gamepad1.left_stick_x;
            double t = -gamepad1.right_stick_x;

            double lf = f - s - t;
            double lr = f + s - t;
            double rf = f + s + t;
            double rr = f - s + t;

            // Keep the wheel ratios when any wheel would go past full power.
            double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lr)),
                    Math.max(Math.abs(rf), Math.abs(rr))));
            double scale = (gamepad1.right_bumper ? SLOW_SCALE : 1.0) / max;

            leftFront.setPower(lf * scale);
            leftRear.setPower(lr * scale);
            rightFront.setPower(rf * scale);
            rightRear.setPower(rr * scale);

            telemetry.addData("Mode", gamepad1.right_bumper ? "slow" : "normal");
            telemetry.addData("Front L/R", "%.2f  %.2f", leftFront.getPower(), rightFront.getPower());
            telemetry.addData("Back  L/R", "%.2f  %.2f", leftRear.getPower(), rightRear.getPower());
            telemetry.update();
        }
    }
}
