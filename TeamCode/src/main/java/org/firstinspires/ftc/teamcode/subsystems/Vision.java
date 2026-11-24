package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Field;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
 * Webcam AprilTags. Every BIOBUZZ tag belongs to a cluster of four on a HIVE cell
 * ("RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING"), and the SDK reports
 * each cluster's pose at the centre of the cell opening, so range and bearing point
 * straight at the cell.
 *
 * Detection is the expensive part, so it only runs while setActive(true). The stream
 * itself stays up so turning detection on is instant.
 */
public class Vision implements Subsystem {

    public static final String CAMERA_NAME = "Webcam 1";

    // Distance band, in inches, the rear outtake scores from.
    public static double SCORING_RANGE_MIN = 46.0;
    public static double SCORING_RANGE_MAX = 62.0;

    // Ignore clusters with fewer than 3 of 4 tags seen, or older than this.
    public static int MIN_CLUSTER_PERCENT = 75;
    public static long MAX_AGE_MS = 200;
    // Weight of each new reading in the smoothed range and bearing.
    public static double SMOOTHING = 0.4;

    /** A HIVE cell to shoot at. Bearing is positive when the cell is to the camera's left. */
    public static class Target {
        public final String name;
        public final double range;
        public final double bearing;

        Target(String name, double range, double bearing) {
            this.name = name;
            this.range = range;
            this.bearing = bearing;
        }

        public boolean inScoringRange() {
            return range >= SCORING_RANGE_MIN && range <= SCORING_RANGE_MAX;
        }
    }

    private final VisionPortal portal;
    private final AprilTagProcessor aprilTag;
    private boolean active;
    private Target smoothed;

    public Vision(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(false)
                .addProcessor(aprilTag)
                .build();
        portal.setProcessorEnabled(aprilTag, false);
    }

    public void setActive(boolean active) {
        if (active != this.active) {
            portal.setProcessorEnabled(aprilTag, active);
            this.active = active;
            smoothed = null;
        }
    }

    /** The closest upright cell of this alliance's colour, smoothed, or null if none is in view. */
    public Target target(Field.Alliance alliance) {
        if (!active) {
            return null;
        }
        AprilTagClusterDetection best = null;
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (!(detection instanceof AprilTagClusterDetection)) {
                continue;
            }
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            if (cluster.ftcPose == null) {
                continue;
            }
            long ageMs = (System.nanoTime() - cluster.frameAcquisitionNanoTime) / 1_000_000L;
            // A cell tilted past 90 degrees is upside down and can't be scored.
            boolean usable = cluster.metadata.name.startsWith(alliance.name())
                    && cluster.percentClusterFound >= MIN_CLUSTER_PERCENT
                    && ageMs <= MAX_AGE_MS
                    && Math.abs(cluster.ftcPose.roll) < 90.0;
            if (usable && (best == null || cluster.ftcPose.range < best.ftcPose.range)) {
                best = cluster;
            }
        }

        if (best == null) {
            smoothed = null;
        } else if (smoothed == null || !smoothed.name.equals(best.metadata.name)) {
            smoothed = new Target(best.metadata.name, best.ftcPose.range, best.ftcPose.bearing);
        } else {
            smoothed = new Target(best.metadata.name,
                    smoothed.range + SMOOTHING * (best.ftcPose.range - smoothed.range),
                    smoothed.bearing + SMOOTHING * (best.ftcPose.bearing - smoothed.bearing));
        }
        return smoothed;
    }

    @Override
    public void update() {}

    @Override
    public void stop() {
        portal.close();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Camera", "%s, detection %s", portal.getCameraState(), active ? "on" : "off");
        if (!active) {
            return;
        }
        telemetry.addData("Camera FPS", "%.0f", portal.getFps());
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection instanceof AprilTagClusterDetection && detection.ftcPose != null) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                telemetry.addData(cluster.metadata.name, "range %.1f in  bearing %.1f°  roll %.0f°  tags %d%%",
                        cluster.ftcPose.range, cluster.ftcPose.bearing, cluster.ftcPose.roll,
                        cluster.percentClusterFound);
            }
        }
    }
}
