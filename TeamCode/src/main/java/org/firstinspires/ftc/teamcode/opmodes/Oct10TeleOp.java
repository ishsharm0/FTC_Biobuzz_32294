package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * gamepad1
 *   left stick     drive / strafe
 *   right stick X  turn
 *   left bumper    slow mode (hold)
 *   left trigger   brake drive wheels (hold)
 *   right bumper   intake (hold)
 *   right trigger  transfer (hold)
 * Motor names and directions must match the robot configuration. If the robot
 * spins or drifts when driving straight, flip the offending motor's direction below.
 */
@TeleOp(name = "Oct10TeleOp", group = "BIOBUZZ")
public class Oct10TeleOp extends LinearOpMode {

    public static double SLOW_SCALE = 0.4;
    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;

    @Override
    public void runOpMode() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor transfer = hardwareMap.get(DcMotor.class, "transfer");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor[] motors = {leftFront, leftRear, rightFront, rightRear, intake, transfer};
        for (DcMotor motor : motors) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Ready");
        telemetry.addLine("LB: slow | LT: drive brake | RB: intake | RT: transfer (hold)");
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
            boolean braking = gamepad1.left_trigger > 0.0;
            boolean slowMode = gamepad1.left_bumper;
            double scale = (slowMode ? SLOW_SCALE : 1.0) / max;

            leftFront.setPower(braking ? 0.0 : lf * scale);
            leftRear.setPower(braking ? 0.0 : lr * scale);
            rightFront.setPower(braking ? 0.0 : rf * scale);
            rightRear.setPower(braking ? 0.0 : rr * scale);

            // Mechanisms operate independently of drive braking and slow mode.
            intake.setPower(gamepad1.right_bumper ? INTAKE_POWER : 0.0);
            transfer.setPower(gamepad1.right_trigger > 0.1 ? TRANSFER_POWER : 0.0);

            telemetry.addData("Mode", braking ? "brake" : slowMode ? "slow" : "normal");
            telemetry.addData("Front L/R", "%.2f  %.2f", leftFront.getPower(), rightFront.getPower());
            telemetry.addData("Back  L/R", "%.2f  %.2f", leftRear.getPower(), rightRear.getPower());
            telemetry.addData("Intake / Transfer", "%.2f  %.2f", intake.getPower(), transfer.getPower());
            telemetry.update();
        }

        for (DcMotor motor : motors) {
            motor.setPower(0.0);
        }
    }
}
