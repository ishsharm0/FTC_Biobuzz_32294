package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * AprilTag demo. The Driver Station shows a top-down radar of every tag the webcam sees,
 * where the target sits in the camera's view, and its distance and angles. The robot can
 * turn to face a tag, or drive up to it and stop square in front of it.
 *
 * To watch the camera itself: during INIT, Driver Station menu → Camera Stream.
 * Tags are outlined, numbered and drawn with their 3D axes.
 *
 * gamepad1
 *   left bumper (hold)    face the target
 *   right bumper (hold)   follow: drive to FOLLOW_DISTANCE in front of the target, square to it
 *   B                     lock onto the next visible tag; after the last, back to "closest"
 *   sticks                drive as in Main TeleOp (after START)
 *   A                     detection on / off
 *   X                     decimation 1-3: higher is faster but sees less far
 *   dpad up / down        exposure: shorter means less blur while moving
 *   dpad right / left     gain: brightens a short exposure
 *   Y                     auto exposure
 */
@TeleOp(name = "AprilTag Demo", group = "BIOBUZZ")
public class AprilTagDemo extends LinearOpMode {

    // Set true if the webcam looks out of the back of the robot, so follow drives the right way.
    public static boolean CAMERA_ON_BACK = false;

    public static double FOLLOW_DISTANCE = 24.0;
    public static double DRIVE_GAIN = 0.03;   // power per inch of distance error
    public static double STRAFE_GAIN = 0.015; // power per degree the tag is turned away
    public static double TURN_GAIN = 0.02;    // power per degree the tag is off centre
    public static double MAX_POWER = 0.4;

    // Radar: robot at the bottom centre, 48 in either side, 96 in ahead.
    private static final int RADAR_COLUMNS = 25;
    private static final int RADAR_ROWS = 10;
    private static final double RADAR_HALF_WIDTH = 48.0;
    private static final double RADAR_DEPTH = 96.0;
    // The view bar spans roughly the webcam's horizontal field of view.
    private static final int BAR_COLUMNS = 31;
    private static final double BAR_HALF_ANGLE = 35.0;

    private static final String TARGET_COLOR = "#4CAF50";
    private static final String OTHER_COLOR = "#2196F3";
    private static final String DIM_COLOR = "#777777";

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
                .build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Vision.CAMERA_NAME))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .build();

        Drivetrain drivetrain = hardwareMap.tryGet(DcMotor.class, Constants.drivetrainConfig.frontLeftName.get()) == null
                ? null
                : new Drivetrain(hardwareMap);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setMsTransmissionInterval(50);

        boolean detecting = true;
        int decimation = 2;
        aprilTag.setDecimation(decimation);
        String locked = null;

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

            List<AprilTagDetection> tags = new ArrayList<>();
            List<AprilTagSingleDetection> unknown = new ArrayList<>();
            for (AprilTagDetection detection : aprilTag.getDetections()) {
                if (detection.ftcPose != null) {
                    tags.add(detection);
                } else if (detection instanceof AprilTagSingleDetection) {
                    unknown.add((AprilTagSingleDetection) detection);
                }
            }
            tags.sort(Comparator.comparingDouble(tag -> tag.ftcPose.range));

            if (gamepad1.bWasPressed()) {
                locked = nextLock(tags, locked);
            }
            AprilTagDetection target = null;
            for (AprilTagDetection tag : tags) {
                if (locked == null || locked.equals(nameOf(tag))) {
                    target = tag;
                    break;
                }
            }

            String mode = "manual";
            if (drivetrain != null && opModeIsActive()) {
                double forward = -gamepad1.left_stick_y;
                double strafe = -gamepad1.left_stick_x;
                double turn = -gamepad1.right_stick_x;
                if (gamepad1.left_bumper || gamepad1.right_bumper) {
                    // Assisted: hold still rather than guess when the target is out of view.
                    forward = strafe = turn = 0.0;
                    mode = target == null ? "target lost, holding still" : "facing";
                    if (target != null) {
                        AprilTagPoseFtc pose = target.ftcPose;
                        // Bearing is positive when the tag is left of centre. Turning left (CCW)
                        // centres it whichever way the camera faces.
                        turn = clip(TURN_GAIN * pose.bearing);
                        if (gamepad1.right_bumper) {
                            double side = CAMERA_ON_BACK ? -1.0 : 1.0;
                            forward = side * clip(DRIVE_GAIN * (pose.range - FOLLOW_DISTANCE));
                            strafe = side * clip(-STRAFE_GAIN * pose.yaw);
                            mode = "following to " + Math.round(FOLLOW_DISTANCE) + " in";
                        }
                    }
                }
                drivetrain.drive(forward, strafe, turn, false);
                drivetrain.update();
            }

            showStatus(detecting, decimation, tags);
            if (detecting) {
                showRadar(tags, target);
                showTarget(target, tags.indexOf(target), locked, mode, drivetrain != null);
                showOthers(tags, target, unknown);
            }
            if (opModeInInit()) {
                telemetry.addLine(dim("LB face · RB follow · B lock · A detect · X decimation · dpad exposure/gain · Y auto"));
            }
            telemetry.update();
        }

        portal.close();
        if (drivetrain != null) {
            drivetrain.stop();
        }
    }

    private void showStatus(boolean detecting, int decimation, List<AprilTagDetection> tags) {
        String latency = tags.isEmpty() ? ""
                : String.format(" · %d ms behind", (System.nanoTime() - tags.get(0).frameAcquisitionNanoTime) / 1_000_000L);
        telemetry.addLine(String.format("<b>AprilTag Demo</b> · %s · %.0f FPS%s",
                portal.getCameraState(), portal.getFps(), latency));
        telemetry.addLine(dim(String.format("detection %s · decimation %d · exposure %s",
                detecting ? "on" : "off", decimation, manualExposure ? exposureMs + " ms, gain " + gain : "auto")));
    }

    /** Top-down view: the robot's camera is the ^ at the bottom, tags are letters. */
    private void showRadar(List<AprilTagDetection> tags, AprilTagDetection target) {
        String[][] grid = new String[RADAR_ROWS][RADAR_COLUMNS];
        for (String[] row : grid) {
            Arrays.fill(row, dim("."));
        }
        grid[RADAR_ROWS - 1][RADAR_COLUMNS / 2] = "<b>^</b>";
        for (int i = 0; i < tags.size(); i++) {
            AprilTagPoseFtc pose = tags.get(i).ftcPose;
            int column = (int) Math.round((pose.x + RADAR_HALF_WIDTH) / (2 * RADAR_HALF_WIDTH) * (RADAR_COLUMNS - 1));
            int row = RADAR_ROWS - 1 - (int) Math.round(pose.y / RADAR_DEPTH * (RADAR_ROWS - 1));
            column = Range.clip(column, 0, RADAR_COLUMNS - 1);
            row = Range.clip(row, 0, RADAR_ROWS - 2);
            grid[row][column] = color(String.valueOf(letter(i)), tags.get(i) == target ? TARGET_COLOR : OTHER_COLOR);
        }
        StringBuilder radar = new StringBuilder("<tt>");
        for (String[] row : grid) {
            radar.append(String.join("", row)).append("<br>");
        }
        telemetry.addLine(radar.append("</tt>").toString());
    }

    private void showTarget(AprilTagDetection target, int index, String locked, String mode, boolean canDrive) {
        if (target == null) {
            telemetry.addLine(locked == null ? "No tags in view" : "Locked on " + locked + ", not in view (B to change)");
            return;
        }
        AprilTagPoseFtc pose = target.ftcPose;

        // Where the target sits across the camera's view.
        int column = (int) Math.round((-pose.bearing + BAR_HALF_ANGLE) / (2 * BAR_HALF_ANGLE) * (BAR_COLUMNS - 1));
        column = Range.clip(column, 0, BAR_COLUMNS - 1);
        StringBuilder bar = new StringBuilder("<tt>[");
        for (int i = 0; i < BAR_COLUMNS; i++) {
            bar.append(i == column ? color("O", TARGET_COLOR) : i == BAR_COLUMNS / 2 ? "|" : dim("-"));
        }
        telemetry.addLine(bar.append("]</tt>").toString());

        telemetry.addLine(String.format("%s %s", color("<b>" + letter(index) + " " + nameOf(target) + "</b>", TARGET_COLOR),
                dim(locked == null ? "(closest)" : "(locked)")));
        telemetry.addLine(String.format("<big>%.1f in</big> · %.1f° %s · %.1f° %s · turned %.0f°",
                pose.range,
                Math.abs(pose.bearing), pose.bearing >= 0 ? "left" : "right",
                Math.abs(pose.elevation), pose.elevation >= 0 ? "up" : "down",
                pose.yaw));
        telemetry.addLine(dim(String.format("x %.1f  y %.1f  z %.1f in · pitch %.0f° · roll %.0f°",
                pose.x, pose.y, pose.z, pose.pitch, pose.roll)));
        if (target instanceof AprilTagClusterDetection) {
            telemetry.addLine(dim(String.format("HIVE cell cluster: %d%% of its tags seen, pose is the cell opening",
                    ((AprilTagClusterDetection) target).percentClusterFound)));
        }
        if (canDrive) {
            telemetry.addLine("Drive: " + mode);
        }
    }

    private void showOthers(List<AprilTagDetection> tags, AprilTagDetection target, List<AprilTagSingleDetection> unknown) {
        for (int i = 0; i < tags.size(); i++) {
            AprilTagDetection tag = tags.get(i);
            if (tag != target) {
                telemetry.addLine(String.format("%s %s · %.0f in", color(String.valueOf(letter(i)), OTHER_COLOR),
                        nameOf(tag), tag.ftcPose.range));
            }
        }
        for (AprilTagSingleDetection tag : unknown) {
            telemetry.addLine(dim(String.format("ID %d: not a game tag, so no distance", tag.id)));
        }
    }

    /** Cycles the lock through the visible tags, nearest first, then back to "closest". */
    private static String nextLock(List<AprilTagDetection> tags, String locked) {
        List<String> names = new ArrayList<>();
        for (AprilTagDetection tag : tags) {
            names.add(nameOf(tag));
        }
        int index = locked == null ? -1 : names.indexOf(locked);
        return index + 1 < names.size() ? names.get(index + 1) : null;
    }

    private static String nameOf(AprilTagDetection detection) {
        if (detection instanceof AprilTagClusterDetection) {
            return ((AprilTagClusterDetection) detection).metadata.name;
        }
        AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
        return tag.metadata == null ? "ID " + tag.id : "ID " + tag.id + " " + tag.metadata.name;
    }

    private static char letter(int index) {
        return (char) ('A' + Math.min(index, 25));
    }

    private static double clip(double power) {
        return Range.clip(power, -MAX_POWER, MAX_POWER);
    }

    private static String color(String text, String color) {
        return "<font color=\"" + color + "\">" + text + "</font>";
    }

    private static String dim(String text) {
        return color(text, DIM_COLOR);
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
