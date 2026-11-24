package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * AprilTag demo. Shows every tag the webcam sees, with its distance and angles, and can
 * turn the robot to face the closest one.
 *
 * To watch the camera: during INIT, open the Driver Station menu → Camera Stream.
 * Tags are outlined, numbered and drawn with their 3D axes and a cube.
 *
 * gamepad1
 *   A               detection on / off (watch the FPS change)
 *   X               cycle decimation: higher is faster but sees tags from less far away
 *   dpad up/down    exposure: shorter means less motion blur while driving
 *   dpad left/right gain: brightens a short exposure
 *   Y               back to auto exposure
 *   after START, if the drive motors are in the config:
 *   left stick / right stick   drive as in Main TeleOp
 *   left bumper (hold)         turn to face the closest tag
 */
@TeleOp(name = "AprilTag Demo", group = "BIOBUZZ")
public class AprilTagDemo extends LinearOpMode {

    // Turn power per degree of bearing when aiming, and its limit.
    public static double AIM_GAIN = 0.02;
    public static double AIM_MAX_POWER = 0.3;

    private VisionPortal portal;
    private boolean manualExposure;
    private long exposureMs = 6;
    private int gain = 100;

    @Override
    public void runOpMode() {
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Vision.CAMERA_NAME))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .build();

        Drivetrain drivetrain = hardwareMap.tryGet(DcMotor.class, Constants.drivetrainConfig.frontLeftName.get()) == null
                ? null
                : new Drivetrain(hardwareMap);

        boolean detecting = true;
        int decimation = 2;
        aprilTag.setDecimation(decimation);

        while (opModeInInit() || opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                detecting = !detecting;
                portal.setProcessorEnabled(aprilTag, detecting);
            }
            if (gamepad1.xWasPressed()) {
                decimation = decimation % 3 + 1;
                aprilTag.setDecimation(decimation);
            }
            adjustCamera();

            List<AprilTagDetection> detections = aprilTag.getDetections();
            AprilTagDetection closest = null;
            for (AprilTagDetection detection : detections) {
                if (detection.ftcPose != null && (closest == null || detection.ftcPose.range < closest.ftcPose.range)) {
                    closest = detection;
                }
            }

            boolean aiming = false;
            if (drivetrain != null && opModeIsActive()) {
                double turn = -gamepad1.right_stick_x;
                if (gamepad1.left_bumper && closest != null) {
                    // Bearing is positive when the tag is left of centre; turning left (CCW) centres it,
                    // whichever way the camera faces.
                    turn = Range.clip(AIM_GAIN * closest.ftcPose.bearing, -AIM_MAX_POWER, AIM_MAX_POWER);
                    aiming = true;
                }
                drivetrain.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, turn, false);
                drivetrain.update();
            }

            telemetry.addData("Camera", "%s  %.0f FPS", portal.getCameraState(), portal.getFps());
            telemetry.addData("Detection", "%s, decimation %d (A, X)", detecting ? "on" : "off", decimation);
            telemetry.addData("Exposure", manualExposure ? exposureMs + " ms, gain " + gain : "auto");
            if (drivetrain == null) {
                telemetry.addLine("No drive motors in config: camera only");
            } else if (aiming) {
                telemetry.addData("Aiming at", "%s, %.1f° off", nameOf(closest), closest.ftcPose.bearing);
            }
            telemetry.addData("Tags seen", detections.size());
            for (AprilTagDetection detection : detections) {
                describe(detection);
            }
            telemetry.update();
        }

        portal.close();
        if (drivetrain != null) {
            drivetrain.stop();
        }
    }

    private void describe(AprilTagDetection detection) {
        if (detection.ftcPose == null) {
            // Only tags in the game's library have a known size, so others get no distance.
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
                telemetry.addLine(String.format("ID %d: not a game tag, no distance (at %.0f, %.0f px)",
                        tag.id, tag.center.x, tag.center.y));
            }
            return;
        }
        telemetry.addLine(nameOf(detection));
        telemetry.addLine(String.format("   %.1f in away, %.1f° %s, %.1f° %s",
                detection.ftcPose.range,
                Math.abs(detection.ftcPose.bearing), detection.ftcPose.bearing >= 0 ? "left" : "right",
                Math.abs(detection.ftcPose.elevation), detection.ftcPose.elevation >= 0 ? "up" : "down"));
        telemetry.addLine(String.format("   x %.1f  y %.1f  z %.1f in   yaw %.0f°  pitch %.0f°  roll %.0f°",
                detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z,
                detection.ftcPose.yaw, detection.ftcPose.pitch, detection.ftcPose.roll));
    }

    private static String nameOf(AprilTagDetection detection) {
        if (detection instanceof AprilTagClusterDetection) {
            // A HIVE cell: its four tags are solved together and the pose is the cell opening.
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            return String.format("%s (cluster, %d%% of tags seen)", cluster.metadata.name, cluster.percentClusterFound);
        }
        AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
        return String.format("ID %d %s", tag.id, tag.metadata == null ? "" : tag.metadata.name);
    }

    private void adjustCamera() {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        GainControl gainControl = portal.getCameraControl(GainControl.class);

        if (gamepad1.yWasPressed()) {
            manualExposure = false;
            exposure.setMode(ExposureControl.Mode.AperturePriority);
            return;
        }
        int exposureStep = gamepad1.dpadUpWasPressed() ? 1 : gamepad1.dpadDownWasPressed() ? -1 : 0;
        int gainStep = gamepad1.dpadRightWasPressed() ? 10 : gamepad1.dpadLeftWasPressed() ? -10 : 0;
        if (exposureStep == 0 && gainStep == 0) {
            return;
        }

        manualExposure = true;
        exposureMs = Math.max(exposure.getMinExposure(TimeUnit.MILLISECONDS),
                Math.min(exposure.getMaxExposure(TimeUnit.MILLISECONDS), exposureMs + exposureStep));
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(exposureMs, TimeUnit.MILLISECONDS);
        if (gainControl != null) {
            gain = Range.clip(gain + gainStep, gainControl.getMinGain(), gainControl.getMaxGain());
            gainControl.setGain(gain);
        }
    }
}
