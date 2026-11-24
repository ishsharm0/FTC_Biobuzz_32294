package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

/*
 * Fallback auto for when odometry isn't working: drives straight forward for a set time
 * to earn LEAVE. Point the robot's front where it should go. Tune DRIVE_SECONDS on the field.
 */
@Autonomous(name = "Timed Leave", group = "BIOBUZZ", preselectTeleOp = "Main TeleOp")
public class TimedLeaveAuto extends LinearOpMode {

    public static double POWER = 0.4;
    public static double DRIVE_SECONDS = 0.6;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        telemetry.addData("Drives forward", "%.1f s at %.0f%% power", DRIVE_SECONDS, POWER * 100);
        telemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive() && getRuntime() < DRIVE_SECONDS) {
            robot.drivetrain.drive(POWER, 0, 0, false);
            robot.update();
        }
        robot.stop();
    }
}
