package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.Vision;

/*
 * Standalone vision test OpMode — no drivetrain, no Robot class.
 * Use this to verify the camera and AprilTag cluster detection independently.
 *
 * Controls:
 *   gamepad1 A   toggle stream on/off  (tests stopStream / resumeStream)
 *   gamepad1 B   toggle processor on/off (tests enable / disable)
 *
 * Logging:
 *   All events and errors are logged to Logcat under the tag "Vision_32294".
 *   In Android Studio Logcat, filter by:  Vision_32294
 *   Any crash prints a full stack trace under:  VisionTest_32294
 *
 * What to look for:
 *   - FPS ~28-32 when streaming and enabled
 *   - RED/BLUE target shows range, bearing, and "SCORABLE" when facing a HIVE
 *   - Bearing near 0° when camera is aimed straight at the CELL opening
 *   - Range matches your tape-measured distance in inches
 *   - Stream and processor toggles work without crashing
 */
@TeleOp(name = "Vision Test", group = "TEST")
public class VisionTest extends LinearOpMode {

    private static final String TAG = "VisionTest_32294";

    @Override
    public void runOpMode() throws InterruptedException {

        RobotLog.ii(TAG, "========== VisionTest INIT ==========");

        Vision vision = null;

        try {
            vision = new Vision(hardwareMap);
        } catch (Exception e) {
            // Camera not found in hardware config, or VisionPortal failed to build.
            RobotLog.ee(TAG, e, "FAILED to create Vision subsystem. " +
                    "Is the webcam named \"%s\" in the Driver Station configuration?",
                    Vision.CAMERA_NAME);
            telemetry.addLine("ERROR: Camera init failed — see Logcat (Vision_32294)");
            telemetry.addData("Detail", e.getMessage());
            telemetry.update();
            // Wait so the driver can read the error before the OpMode ends.
            while (!isStarted() && !isStopRequested()) { sleep(100); }
            return;
        }

        // Wait for camera to start streaming.
        telemetry.addLine("Waiting for camera...");
        telemetry.update();
        vision.waitUntilReady();

        if (vision.isReady()) {
            RobotLog.ii(TAG, "Camera ready. Waiting for Play.");
            telemetry.addLine("Camera READY — press Play to start");
        } else {
            RobotLog.ee(TAG, "Camera did not reach STREAMING state during init.");
            telemetry.addLine("WARNING: Camera timeout — check USB cable and config name");
        }
        telemetry.update();

        waitForStart();
        RobotLog.ii(TAG, "========== VisionTest START ==========");

        while (opModeIsActive()) {
            try {

                // ── Controls ──────────────────────────────────────────────────
                if (gamepad1.aWasPressed()) {
                    if (vision.isStreaming()) {
                        RobotLog.ii(TAG, "A pressed — stopping stream.");
                        vision.stopStream();
                    } else {
                        RobotLog.ii(TAG, "A pressed — resuming stream.");
                        vision.resumeStream();
                    }
                }

                if (gamepad1.bWasPressed()) {
                    if (vision.isEnabled()) {
                        RobotLog.ii(TAG, "B pressed — disabling processor.");
                        vision.disable();
                    } else {
                        RobotLog.ii(TAG, "B pressed — enabling processor.");
                        vision.enable();
                    }
                }

                // ── Targets ───────────────────────────────────────────────────
                Vision.HiveTarget red  = vision.getHiveTarget(Vision.Alliance.RED);
                Vision.HiveTarget blue = vision.getHiveTarget(Vision.Alliance.BLUE);

                // ── Telemetry ─────────────────────────────────────────────────
                telemetry.addLine("── Controls ─────────────────────────");
                telemetry.addData("A", "toggle stream  |  B: toggle processor");

                telemetry.addLine("── Status ───────────────────────────");
                telemetry.addData("Streaming",  vision.isStreaming());
                telemetry.addData("Processor",  vision.isEnabled() ? "enabled" : "disabled");

                vision.addTelemetry(telemetry);

                telemetry.addLine("── Targets ──────────────────────────");
                telemetry.addData("RED  target",  red  != null ? red.toString()  : "none");
                telemetry.addData("BLUE target",  blue != null ? blue.toString() : "none");

                if (red != null) {
                    telemetry.addData("RED  in window",
                            red.isInScoringWindow()
                                    ? "YES — SHOOT"
                                    : "no  (range error " + String.format("%.1f", red.rangeError()) + "\")");
                }
                if (blue != null) {
                    telemetry.addData("BLUE in window",
                            blue.isInScoringWindow()
                                    ? "YES — SHOOT"
                                    : "no  (range error " + String.format("%.1f", blue.rangeError()) + "\")");
                }

                telemetry.update();

            } catch (Exception e) {
                // Catch anything unexpected inside the loop so we get a full trace.
                RobotLog.ee(TAG, e, "Unexpected exception in VisionTest loop.");
                telemetry.addLine("ERROR in loop — see Logcat (VisionTest_32294)");
                telemetry.addData("Detail", e.getMessage());
                telemetry.update();
                // Brief pause before continuing so telemetry is readable.
                sleep(500);
            }
        }

        RobotLog.ii(TAG, "========== VisionTest STOP ==========");
        vision.stop();
    }
}
