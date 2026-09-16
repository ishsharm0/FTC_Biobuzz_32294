package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/*
 * gamepad1
 *   left stick     drive / strafe
 *   right stick X  turn
 *   right bumper   slow mode (hold)
 *   back           toggle field-centric
 *   start          reset heading (field-centric forward = where the robot faces now)
 *   dpad up        toggle velocity drive (steady wheel speed) vs follower drive
 */
@TeleOp(name = "Main TeleOp", group = "BIOBUZZ")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        robot.drivetrain.startTeleOp(Robot.lastPose);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.backWasPressed()) {
                robot.drivetrain.setFieldCentric(!robot.drivetrain.isFieldCentric());
            }
            if (gamepad1.startWasPressed()) {
                robot.drivetrain.resetHeading();
            }
            if (gamepad1.dpadUpWasPressed()) {
                robot.drivetrain.setMode(robot.drivetrain.getMode() == Drivetrain.Mode.PEDRO
                        ? Drivetrain.Mode.VELOCITY
                        : Drivetrain.Mode.PEDRO);
            }

            robot.drivetrain.setSlowMode(gamepad1.right_bumper);
            robot.drivetrain.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            robot.update();

            robot.addTelemetry(telemetry);
            telemetry.update();
        }

        robot.stop();
    }
}
