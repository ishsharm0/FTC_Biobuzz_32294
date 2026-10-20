package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/*
 * gamepad1
 *   left stick      drive / strafe
 *   right stick X   turn
 *   right bumper    slow mode (hold)
 *   left bumper     vision aim mode (hold) — enables camera while held, disables on release
 *   back            toggle field-centric
 *   start           reset heading (field-centric forward = where the robot faces now)
 *   dpad up         toggle velocity drive (steady wheel speed) vs follower drive
 *
 * Vision CPU strategy:
 *   Camera stream is stopped on match start to save CPU.
 *   Holding left bumper resumes the stream and enables the AprilTag processor.
 *   Releasing left bumper disables the processor.
 *   The stream stays running while left bumper is held, and is stopped again
 *   1 second after the bumper is released to avoid thrashing start/stop rapidly.
 *
 * IMPORTANT: Change ALLIANCE below to match your alliance before the match.
 */
@TeleOp(name = "Main TeleOp", group = "BIOBUZZ")
public class MainTeleOp extends LinearOpMode {

    // ── Set this to your alliance before the match ────────────────────────────
    private static final Vision.Alliance ALLIANCE = Vision.Alliance.RED;

    // How long after releasing the aim button before the stream is fully stopped (ms).
    // Keeps it warm briefly so quick re-presses don't pay the resume cost.
    private static final long STREAM_IDLE_TIMEOUT_MS = 1000;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        robot.drivetrain.startTeleOp(Robot.lastPose);

        // Stop the camera stream immediately — saves CPU for the whole match.
        // It will be resumed on demand when the driver holds left bumper.
        robot.vision.stopStream();

        telemetry.addLine("Ready  |  hold LB to aim at HIVE");
        telemetry.update();

        waitForStart();

        long streamIdleStart = -1; // timestamp when aim button was last released

        while (opModeIsActive()) {

            // ── Drivetrain controls ───────────────────────────────────────────
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
            robot.drivetrain.drive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x);

            // ── Vision aim mode (left bumper) ─────────────────────────────────
            if (gamepad1.left_bumper) {
                // Button held — ensure stream and processor are both on.
                if (!robot.vision.isStreaming()) {
                    robot.vision.resumeStream();
                }
                robot.vision.enable();
                streamIdleStart = -1; // reset idle timer

                Vision.HiveTarget target = robot.vision.getHiveTarget(ALLIANCE);
                if (target != null) {
                    telemetry.addData("HIVE Target", target.toString());
                    telemetry.addData("In window", target.isInScoringWindow() ? "YES — SHOOT" : "no");
                } else {
                    telemetry.addData("HIVE Target", "not found");
                }
            } else {
                // Button released — disable processor immediately to free CPU.
                if (robot.vision.isEnabled()) {
                    robot.vision.disable();
                    streamIdleStart = System.currentTimeMillis();
                }

                // After idle timeout, stop the stream entirely.
                if (streamIdleStart > 0
                        && robot.vision.isStreaming()
                        && System.currentTimeMillis() - streamIdleStart > STREAM_IDLE_TIMEOUT_MS) {
                    robot.vision.stopStream();
                    streamIdleStart = -1;
                }
            }

            // ── Update and telemetry ──────────────────────────────────────────
            robot.update();
            robot.addTelemetry(telemetry);
            telemetry.update();
        }

        robot.stop();
    }
}
