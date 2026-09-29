package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
 * Fallback drive: plain motor power, no Pedro follower, no Pinpoint.
 * Use it when odometry is unplugged or misbehaving. Wheel encoder counts are shown
 * on telemetry for checking motors and wiring.
 *
 * gamepad1
 *   left stick     drive / strafe
 *   right stick X  turn
 *   right bumper   slow mode (hold)
 *   start          zero the encoder counts
 */
@TeleOp(name = "Motor Only TeleOp", group = "BIOBUZZ")
public class MotorOnlyTeleOp extends LinearOpMode {

    public static double SLOW_SCALE = 0.4;

    @Override
    public void runOpMode() {
        MecanumConfig config = Constants.drivetrainConfig;
        DcMotor leftFront = hardwareMap.get(DcMotor.class, config.frontLeftName.get());
        DcMotor leftRear = hardwareMap.get(DcMotor.class, config.backLeftName.get());
        DcMotor rightFront = hardwareMap.get(DcMotor.class, config.frontRightName.get());
        DcMotor rightRear = hardwareMap.get(DcMotor.class, config.backRightName.get());
        DcMotor[] motors = {leftFront, leftRear, rightFront, rightRear};

        leftFront.setDirection(config.frontLeftDirection.get());
        leftRear.setDirection(config.backLeftDirection.get());
        rightFront.setDirection(config.frontRightDirection.get());
        rightRear.setDirection(config.backRightDirection.get());

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        resetEncoders(motors);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.startWasPressed()) {
                resetEncoders(motors);
            }

            double scale = gamepad1.right_bumper ? SLOW_SCALE : 1.0;
            double f = -gamepad1.left_stick_y;
            double s = -gamepad1.left_stick_x;
            double t = -gamepad1.right_stick_x;

            double lf = f - s - t;
            double lr = f + s - t;
            double rf = f + s + t;
            double rr = f - s + t;

            double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lr)),
                    Math.max(Math.abs(rf), Math.abs(rr))));

            leftFront.setPower(lf / max * scale);
            leftRear.setPower(lr / max * scale);
            rightFront.setPower(rf / max * scale);
            rightRear.setPower(rr / max * scale);

            telemetry.addData("Mode", gamepad1.right_bumper ? "slow" : "normal");
            telemetry.addData("Front L/R", "%d  %d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
            telemetry.addData("Back  L/R", "%d  %d", leftRear.getCurrentPosition(), rightRear.getCurrentPosition());
            telemetry.update();
        }

        for (DcMotor motor : motors) {
            motor.setPower(0.0);
        }
    }

    private static void resetEncoders(DcMotor[] motors) {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
