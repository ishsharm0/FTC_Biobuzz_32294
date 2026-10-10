package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Simple timed leave: scoot forward, pause, then reverse approximately to the start.
 * Only needs the four drive motors; no odometry, Pinpoint, or Pedro follower.
 * Adjust DRIVE_POWER and DRIVE_TIME_MS on the robot to set the travel distance.
 */
@Autonomous(name = "Leave", group = "BIOBUZZ")
public class LeaveAuto extends LinearOpMode {

    public static double DRIVE_POWER = 0.3;
    public static long DRIVE_TIME_MS = 600;
    public static long PAUSE_MS = 200;

    private DcMotor[] motors;

    @Override
    public void runOpMode() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "backRight");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        motors = new DcMotor[]{leftFront, leftRear, rightFront, rightRear};
        for (DcMotor motor : motors) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Ready: forward, pause, backward, stop.");
        telemetry.addData("Power / Time", "%.2f / %d ms each way", DRIVE_POWER, DRIVE_TIME_MS);
        telemetry.update();

        waitForStart();

        try {
            driveFor(DRIVE_POWER, DRIVE_TIME_MS, "Forward");
            if (!opModeIsActive()) {
                return;
            }
            sleep(PAUSE_MS);
            driveFor(-DRIVE_POWER, DRIVE_TIME_MS, "Backward");

            telemetry.addLine("Done");
            telemetry.update();
        } finally {
            setDrivePower(0.0);
        }
    }

    private void driveFor(double power, long durationMs, String step) {
        if (!opModeIsActive()) {
            return;
        }
        telemetry.addData("Step", step);
        telemetry.update();

        setDrivePower(power);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < durationMs) {
            idle();
        }
        setDrivePower(0.0);
    }

    private void setDrivePower(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }
}
