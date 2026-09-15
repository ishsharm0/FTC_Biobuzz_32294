package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * Robot-centric mecanum drive.
 *
 * Left stick: drive forward/back and strafe. Right stick X: turn.
 * Hold right bumper for slow mode.
 *
 * Motor names must match the robot configuration on the Driver Station:
 * frontLeft, frontRight, backLeft, backRight.
 * If the robot drives backwards or spins when it should go straight,
 * flip the direction of the offending motor(s) in init.
 */
@TeleOp(name = "Mecanum TeleOp", group = "BIOBUZZ")
public class MecanumTeleOp extends LinearOpMode {

    private static final double SLOW_SCALE = 0.4;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Stick Y is negative when pushed forward.
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double fl = drive + strafe + turn;
            double fr = drive - strafe - turn;
            double bl = drive - strafe + turn;
            double br = drive + strafe - turn;

            // Scale down so no wheel exceeds full power while keeping the ratios.
            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            double scale = gamepad1.right_bumper ? SLOW_SCALE : 1.0;

            frontLeft.setPower(fl / max * scale);
            frontRight.setPower(fr / max * scale);
            backLeft.setPower(bl / max * scale);
            backRight.setPower(br / max * scale);

            telemetry.addData("Mode", gamepad1.right_bumper ? "slow" : "normal");
            telemetry.addData("Front L/R", "%.2f  %.2f", frontLeft.getPower(), frontRight.getPower());
            telemetry.addData("Back  L/R", "%.2f  %.2f", backLeft.getPower(), backRight.getPower());
            telemetry.update();
        }
    }
}
