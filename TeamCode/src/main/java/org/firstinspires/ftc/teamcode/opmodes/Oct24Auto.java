package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Oct24Auto — Minimum points AUTO (no Pedro, timed drive).
 *
 * Path:
 *   1. Drive forward off starting tile     → AUTO LEAVE  (3 pts)
 *   2. Continue forward to GARDEN corner   → parked in GARDEN
 *
 * Tune LEAVE_TIME_MS and GARDEN_TIME_MS on the practice field with a tape measure.
 * Increase the time to go farther, decrease to stop sooner.
 */
@Autonomous(name = "Oct24 Auto", group = "BIOBUZZ")
public class Oct24Auto extends LinearOpMode {

    // Power to drive forward (0.0 - 1.0). Lower = more control, easier to tune.
    public static double DRIVE_POWER = 0.4;

    // Time to drive just past the starting tile edge — earns AUTO LEAVE (3 pts).
    public static long LEAVE_TIME_MS = 600;

    // Additional time to continue forward into the GARDEN corner.
    public static long GARDEN_TIME_MS = 2000;

    @Override
    public void runOpMode() {
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

        telemetry.addLine("Oct24 Auto ready — LEAVE then GARDEN");
        telemetry.addData("Power", DRIVE_POWER);
        telemetry.addData("Leave time",  LEAVE_TIME_MS  + " ms");
        telemetry.addData("Garden time", GARDEN_TIME_MS + " ms");
        telemetry.update();

        waitForStart();

        // ── Step 1: Leave starting tile (AUTO LEAVE = 3 pts) ─────────────────
        telemetry.addLine("Step 1: Leaving starting tile...");
        telemetry.update();
        driveFor(motors, DRIVE_POWER, LEAVE_TIME_MS);

        if (!opModeIsActive()) return;

        // ── Step 2: Continue to GARDEN ────────────────────────────────────────
        telemetry.addLine("Step 2: Driving to GARDEN...");
        telemetry.update();
        driveFor(motors, DRIVE_POWER, GARDEN_TIME_MS);

        // ── Done: parked in GARDEN ────────────────────────────────────────────
        telemetry.addLine("Done — parked in GARDEN");
        telemetry.update();
    }

    private void driveFor(DcMotor[] motors, double power, long durationMs) {
        for (DcMotor motor : motors) motor.setPower(power);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < durationMs) {
            idle();
        }
        for (DcMotor motor : motors) motor.setPower(0.0);
    }
}
