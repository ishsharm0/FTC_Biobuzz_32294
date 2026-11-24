package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/*
 * Robot-centric: pushing the left stick up always drives toward the robot's front.
 *
 * gamepad1
 *   left stick      drive / strafe
 *   right stick X   turn
 *   left trigger    slow mode (hold)
 *   right bumper    intake in (hold)
 *   left bumper     intake out (hold)
 *   A               feed the rear launcher (hold)
 *   Y               show the HIVE target from the camera (hold)
 */
@TeleOp(name = "Main TeleOp", group = "BIOBUZZ")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        telemetry.addData("Alliance", Robot.alliance);
        telemetry.addData("Camera", robot.vision == null ? "not in config" : "ready, hold Y to aim");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.drivetrain.drive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    gamepad1.left_trigger > 0.5);

            robot.intake.set(gamepad1.right_bumper ? 1 : gamepad1.left_bumper ? -1 : 0);
            robot.launcher.feed(gamepad1.a);

            if (robot.vision != null) {
                robot.vision.setActive(gamepad1.y);
                Vision.Target target = robot.vision.target(Robot.alliance);
                if (target != null) {
                    telemetry.addData("HIVE", "%s  %.0f in  %.0f°  %s", target.name, target.range, target.bearing,
                            target.inScoringRange() ? "IN RANGE" : "");
                }
            }

            robot.update();
            robot.addTelemetry(telemetry);
            telemetry.update();
        }

        robot.stop();
    }
}
