package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * Vision subsystem — Logitech C920 webcam, FTC SDK VisionPortal.
 *
 * ── BIOBUZZ AprilTag layout ──────────────────────────────────────────────────
 * All 16 BIOBUZZ AprilTags are in 4 clusters of 4 tags each.
 * There are NO single AprilTags in BIOBUZZ — all detections are AprilTagClusterDetection.
 *
 *   Cluster name       Tag IDs    Location
 *   "RED SCORING"      30-33      Red HIVE, scoring-table side
 *   "RED AUDIENCE"     34-37      Red HIVE, audience side
 *   "BLUE AUDIENCE"    38-41      Blue HIVE, audience side
 *   "BLUE SCORING"     42-45      Blue HIVE, scoring-table side
 *
 * Each cluster's ftcPose points to the CENTER OF THE HIVE CELL OPENING — not to
 * the tag itself. ftcPose.range and ftcPose.bearing give the distance and angle
 * to the CELL opening directly.
 *
 * ── CPU management — IMPORTANT ───────────────────────────────────────────────
 * The C920 runs on the Control Hub's shared CPU. Running vision continuously
 * throughout the match wastes CPU that the drivetrain, odometry, and other
 * subsystems need. Two levels of control are provided:
 *
 *   enable() / disable()
 *     Toggles the AprilTag processor on/off while keeping the camera streaming.
 *     Fast to switch (~0 ms). Use this for quick on/off during a match loop.
 *     Camera stays warm so re-enabling is instant.
 *     CPU cost when disabled: minimal (camera streams but no detection runs).
 *
 *   resumeStream() / stopStream()
 *     Fully starts/stops the camera. Maximum CPU savings when stopped —
 *     zero frames are captured or processed.
 *     Takes ~200-500 ms to resume, so plan ahead (call resumeStream()
 *     before you need a detection, not at the same moment).
 *     Use this during long periods where vision is completely unneeded.
 *
 * Recommended usage pattern:
 *
 *   AUTO init:    camera starts streaming (default), waitUntilReady(), read target → stopStream()
 *   AUTO match:   resumeStream() ~1s before approaching HIVE → read target → stopStream()
 *   TELEOP idle:  stream stopped or processor disabled
 *   TELEOP aim:   enable() → read target each loop → disable() after shot
 *
 * ── Primary use cases ────────────────────────────────────────────────────────
 * 1. HIVE aiming — call enable(), then getHiveTarget(alliance) each loop.
 *    Use HiveTarget.bearing to steer (toward 0°) and HiveTarget.range
 *    to know when you are in the scoring window (46-62 inches).
 *
 * Hardware config name: "Webcam 1"  (change CAMERA_NAME if yours differs).
 */
public class Vision implements Subsystem {

    // ── Hardware config name ──────────────────────────────────────────────────
    public static final String CAMERA_NAME = "Webcam 1";

    // ── Logcat tag — filter by this in Android Studio Logcat ─────────────────
    private static final String TAG = "Vision_32294";

    // ── Camera resolution ─────────────────────────────────────────────────────
    private static final int FRAME_WIDTH  = 640;
    private static final int FRAME_HEIGHT = 480;

    // ── Scoring window ────────────────────────────────────────────────────────
    // From the FTC skill: optimal launch window is 46-62 inches from the HIVE.
    public static final double SCORING_WINDOW_NEAR_IN  = 46.0;
    public static final double SCORING_WINDOW_FAR_IN   = 62.0;
    public static final double SCORING_WINDOW_IDEAL_IN =
            (SCORING_WINDOW_NEAR_IN + SCORING_WINDOW_FAR_IN) / 2.0;

    // ── Roll threshold for scorable CELL detection ────────────────────────────
    // |roll| < 90° → cluster is right-side up → CELL is scorable.
    private static final double SCORABLE_ROLL_THRESHOLD_DEG = 90.0;

    // ── Alliance color ────────────────────────────────────────────────────────
    public enum Alliance { RED, BLUE }

    // ── HiveTarget — result returned to callers ───────────────────────────────
    /**
     * All information needed to aim at and drive toward a HIVE CELL.
     * Obtain via {@link #getHiveTarget(Alliance)}.
     */
    public static class HiveTarget {
        /** Cluster name, e.g. "RED AUDIENCE". */
        public final String name;
        /** Distance to the CELL opening in inches. */
        public final double range;
        /**
         * Horizontal angle to the CELL opening in degrees.
         * Positive = CELL is to the robot's LEFT; negative = RIGHT.
         * Steer until this is close to 0° to be aimed straight at the CELL.
         */
        public final double bearing;
        /** Fraction of the cluster's 4 tags that were detected (0–100). Higher = more reliable. */
        public final int percentFound;

        HiveTarget(String name, double range, double bearing, int percentFound) {
            this.name         = name;
            this.range        = range;
            this.bearing      = bearing;
            this.percentFound = percentFound;
        }

        /** True if the robot is within the optimal BIOBUZZ scoring window (46–62 in). */
        public boolean isInScoringWindow() {
            return range >= SCORING_WINDOW_NEAR_IN && range <= SCORING_WINDOW_FAR_IN;
        }

        /**
         * Signed distance error from the ideal shooting distance.
         * Positive = too far, negative = too close, zero = ideal.
         * Multiply by a gain constant to get a forward drive power.
         */
        public double rangeError() {
            return range - SCORING_WINDOW_IDEAL_IN;
        }

        @Override
        public String toString() {
            return String.format("%s  range=%.1f\"  bearing=%.1f°  tags=%d%%",
                    name, range, bearing, percentFound);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────

    private final VisionPortal      visionPortal;
    private final AprilTagProcessor aprilTag;

    // Tracks whether the processor is currently enabled.
    // Mirrors the state of setProcessorEnabled() so callers can query it cheaply.
    private boolean processorEnabled = true;

    // Tracks whether the camera stream is running.
    private boolean streaming = true;

    public Vision(HardwareMap hardwareMap) {
        RobotLog.ii(TAG, "Initialising — camera config name: \"%s\"  resolution: %dx%d",
                CAMERA_NAME, FRAME_WIDTH, FRAME_HEIGHT);

        // Check the webcam is present in the hardware map before building the portal.
        // A missing config name throws an IllegalArgumentException that is hard to diagnose.
        if (hardwareMap.tryGet(WebcamName.class, CAMERA_NAME) == null) {
            RobotLog.ee(TAG, "Camera \"%s\" NOT found in hardware config. " +
                    "Check the name in the Driver Station configuration matches CAMERA_NAME.",
                    CAMERA_NAME);
        }

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .addProcessor(aprilTag)
                .setCameraResolution(new android.util.Size(FRAME_WIDTH, FRAME_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();

        RobotLog.ii(TAG, "VisionPortal built — waiting for streaming to begin.");
    }

    // ── Subsystem ─────────────────────────────────────────────────────────────

    @Override
    public void update() {
        // VisionPortal runs detection on its own background thread.
        // Nothing to do here — results are available on demand via getHiveTarget().
    }

    @Override
    public void stop() {
        RobotLog.ii(TAG, "Closing VisionPortal.");
        visionPortal.close();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Vision/Camera",
                streaming ? visionPortal.getCameraState().toString() : "STREAM STOPPED");
        telemetry.addData("Vision/Processor", processorEnabled ? "enabled" : "disabled");

        if (streaming && processorEnabled) {
            telemetry.addData("Vision/FPS", "%.1f", visionPortal.getFps());

            List<AprilTagDetection> detections = aprilTag.getDetections();
            int clusterCount = 0;

            for (AprilTagDetection d : detections) {
                if (!(d instanceof AprilTagClusterDetection)) continue;
                AprilTagClusterDetection cd = (AprilTagClusterDetection) d;
                clusterCount++;

                telemetry.addData(
                        "Vision/" + cd.metadata.name,
                        "range=%.1f\"  bear=%.1f°  roll=%.1f°  tags=%d%%  %s",
                        cd.ftcPose.range,
                        cd.ftcPose.bearing,
                        cd.ftcPose.roll,
                        cd.percentClusterFound,
                        isScorableCell(cd) ? "SCORABLE" : "not scorable");
            }

            if (clusterCount == 0) {
                telemetry.addData("Vision/Clusters", "none seen");
            }
        }
    }

    // ── Camera readiness ──────────────────────────────────────────────────────

    /** True once the camera has started streaming frames. */
    public boolean isReady() {
        return visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /**
     * Blocks until the camera has started streaming, checking every 50 ms.
     * Times out after 5 seconds. Safe to call from init().
     */
    public void waitUntilReady() {
        long deadline = System.currentTimeMillis() + 5000;
        while (!isReady() && System.currentTimeMillis() < deadline) {
            try { Thread.sleep(50); } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                RobotLog.ww(TAG, e, "waitUntilReady interrupted before camera was ready.");
                break;
            }
        }

        if (isReady()) {
            RobotLog.ii(TAG, "Camera is STREAMING and ready.");
        } else {
            RobotLog.ee(TAG, "Camera did NOT reach STREAMING state within 5 seconds. " +
                    "Current state: %s. Check USB connection and hardware config.",
                    visionPortal.getCameraState());
        }
    }

    // ── CPU management ────────────────────────────────────────────────────────

    /**
     * Enables the AprilTag processor. Camera continues streaming; detection resumes instantly.
     * Call this just before a loop that reads {@link #getHiveTarget(Alliance)}.
     */
    public void enable() {
        if (!processorEnabled) {
            visionPortal.setProcessorEnabled(aprilTag, true);
            processorEnabled = true;
            RobotLog.ii(TAG, "AprilTag processor ENABLED.");
        }
    }

    /**
     * Disables the AprilTag processor. Camera stays streaming but no detection runs.
     * CPU cost drops significantly. Re-enabling via {@link #enable()} is instant.
     * Call this after you have finished reading targets for a cycle.
     */
    public void disable() {
        if (processorEnabled) {
            visionPortal.setProcessorEnabled(aprilTag, false);
            processorEnabled = false;
            RobotLog.ii(TAG, "AprilTag processor DISABLED.");
        }
    }

    /** True if the AprilTag processor is currently enabled. */
    public boolean isEnabled() {
        return processorEnabled;
    }

    /**
     * Fully stops the camera stream. Maximum CPU savings — zero frames are captured.
     * Use during long stretches where vision is completely unneeded.
     *
     * WARNING: resumeStream() takes ~200–500 ms to be ready. Call it well before
     * you need a detection — not at the same moment you want to read a target.
     */
    public void stopStream() {
        if (streaming) {
            RobotLog.ii(TAG, "Stopping camera stream.");
            visionPortal.stopStreaming();
            streaming = false;
        }
    }

    /**
     * Resumes the camera stream after {@link #stopStream()}.
     * Blocks briefly until streaming is confirmed or 2 seconds elapse.
     * Also re-enables the processor if it was enabled before the stream was stopped.
     */
    public void resumeStream() {
        if (!streaming) {
            RobotLog.ii(TAG, "Resuming camera stream...");
            visionPortal.resumeStreaming();
            streaming = true;

            // Wait until the camera is actually streaming before returning.
            long deadline = System.currentTimeMillis() + 2000;
            while (!isReady() && System.currentTimeMillis() < deadline) {
                try { Thread.sleep(30); } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    RobotLog.ww(TAG, e, "resumeStream interrupted while waiting for STREAMING state.");
                    break;
                }
            }

            if (isReady()) {
                RobotLog.ii(TAG, "Camera stream RESUMED and streaming.");
            } else {
                RobotLog.ee(TAG, "Camera did NOT return to STREAMING after resumeStream(). " +
                        "Current state: %s.", visionPortal.getCameraState());
            }
        }
    }

    /** True if the camera stream is currently running. */
    public boolean isStreaming() {
        return streaming;
    }

    // ── HIVE targeting ────────────────────────────────────────────────────────

    /**
     * Returns a {@link HiveTarget} for the best scorable HIVE CELL for the given alliance.
     *
     * Requires the processor to be enabled and streaming to be active.
     * Returns {@code null} if:
     *   - vision is disabled or stream is stopped
     *   - no clusters are visible for the given alliance
     *   - all visible clusters are upside-down (not scorable)
     *
     * If multiple scorable clusters are visible, the closest one is returned.
     *
     * Example usage in a match loop:
     * <pre>
     *   robot.vision.enable();
     *   HiveTarget target = robot.vision.getHiveTarget(Vision.Alliance.RED);
     *   if (target != null && target.isInScoringWindow()) {
     *       // shoot
     *   } else if (target != null) {
     *       // steer toward bearing=0, drive toward rangeError()=0
     *   }
     *   robot.vision.disable(); // call after shot cycle is complete
     * </pre>
     */
    public HiveTarget getHiveTarget(Alliance alliance) {
        if (!processorEnabled || !streaming) {
            RobotLog.ww(TAG, "getHiveTarget() called but vision is not active " +
                    "(streaming=%b, processorEnabled=%b). " +
                    "Call resumeStream() and enable() before reading targets.",
                    streaming, processorEnabled);
            return null;
        }

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) return null;

        String prefix = (alliance == Alliance.RED) ? "R" : "B";
        HiveTarget best = null;

        for (AprilTagDetection d : detections) {
            if (!(d instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection cd = (AprilTagClusterDetection) d;

            // Filter by alliance color.
            if (!cd.metadata.name.startsWith(prefix)) continue;

            // Filter to scorable cells only — cluster must be right-side up.
            if (!isScorableCell(cd)) continue;

            HiveTarget candidate = new HiveTarget(
                    cd.metadata.name,
                    cd.ftcPose.range,
                    cd.ftcPose.bearing,
                    cd.percentClusterFound);

            // Pick the closest valid target.
            if (best == null || candidate.range < best.range) {
                best = candidate;
            }
        }

        if (best != null) {
            RobotLog.dd(TAG, "getHiveTarget(%s): %s", alliance, best);
        }

        return best;
    }

    /** Convenience: true if a scorable HIVE CELL is currently visible for the given alliance. */
    public boolean hasHiveTarget(Alliance alliance) {
        return getHiveTarget(alliance) != null;
    }

    /** Returns the raw list of all current AprilTag detections for custom processing. */
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    // ── Internal ──────────────────────────────────────────────────────────────

    /**
     * A CELL is scorable when its cluster is right-side up: |roll| < 90°.
     * See FTC Tech Tip: AprilTag Clusters for the official rule.
     */
    private static boolean isScorableCell(AprilTagClusterDetection cd) {
        return Math.abs(cd.ftcPose.roll) < SCORABLE_ROLL_THRESHOLD_DEG;
    }
}
