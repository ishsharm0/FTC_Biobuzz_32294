package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayDeque;
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
 * ── Accuracy improvements ────────────────────────────────────────────────────
 * 1. Confidence filter— rejects detections where fewer than MIN_CLUSTER_PERCENT of
 *                       the cluster's tags were found.
 * 2. Rolling average  — smooths bearing and range over the last N frames to reduce
 *                       jitter, per alliance.
 * 3. Age check        — discards detections older than MAX_DETECTION_AGE_MS.
 * 4. Lens calibration — setLensIntrinsics() gives the SDK accurate focal length and
 *                       principal point for the C920, improving range/bearing accuracy.
 *
 * ── CPU management ───────────────────────────────────────────────────────────
 *   enable() / disable()     — toggles processor on/off, camera stays warm
 *   stopStream() / resumeStream() — fully stops/starts camera for max CPU savings
 *
 * Hardware config name: "Webcam 1"  (change CAMERA_NAME if yours differs).
 */
public class Vision implements Subsystem {

    // ── Hardware config name ──────────────────────────────────────────────────
    public static final String CAMERA_NAME = "Webcam 1";

    // ── Logcat tag ────────────────────────────────────────────────────────────
    private static final String TAG = "Vision_32294";

    // ── Camera resolution ─────────────────────────────────────────────────────
    private static final int FRAME_WIDTH  = 640;
    private static final int FRAME_HEIGHT = 480;

    // ── Accuracy: 2. Confidence filter ───────────────────────────────────────
    // Reject cluster detections where fewer than this % of tags were found.
    // 75 = at least 3 of 4 tags visible. Raise to 100 for strictest filtering.
    public static int MIN_CLUSTER_PERCENT = 75;

    // ── Accuracy: 3. Rolling average ─────────────────────────────────────────
    // Number of frames to average bearing and range over per alliance.
    // Higher = smoother but slower to respond to movement.
    private static final int ROLLING_AVERAGE_SIZE = 5;

    // ── Accuracy: 4. Detection age check ─────────────────────────────────────
    // Discard detections older than this. Prevents acting on stale data
    // when the robot has moved since the last frame was captured.
    public static long MAX_DETECTION_AGE_MS = 200;

    // ── Accuracy: 4. Lens calibration (ADESSO HD 1080p at 640x480) ───────────
    // No published calibration exists for the ADESSO. These are safe generic
    // estimates for a 1080p sensor run at 640x480:
    //   fx = fy = FRAME_WIDTH (standard approximation for unknown webcams)
    //   cx = FRAME_WIDTH / 2, cy = FRAME_HEIGHT / 2 (optical centre)
    //
    // For best accuracy, run a ChArUco calibration on your specific camera:
    //   1. Print the ChArUco board from the Limelight calibration page
    //   2. Run the FTC SDK calibration OpMode (ConceptAprilTagOptimizeExposure)
    //   3. Replace the values below with your results
    //
    // If USE_CALIBRATION = false, the SDK uses its own built-in defaults.
    private static final boolean USE_CALIBRATION = true;
    private static final double CAL_FX = 640.0;   // generic estimate: equal to frame width
    private static final double CAL_FY = 640.0;
    private static final double CAL_CX = 320.0;   // half of FRAME_WIDTH
    private static final double CAL_CY = 240.0;   // half of FRAME_HEIGHT

    // ── Scoring window ────────────────────────────────────────────────────────
    public static final double SCORING_WINDOW_NEAR_IN  = 46.0;
    public static final double SCORING_WINDOW_FAR_IN   = 62.0;
    public static final double SCORING_WINDOW_IDEAL_IN =
            (SCORING_WINDOW_NEAR_IN + SCORING_WINDOW_FAR_IN) / 2.0;

    // ── Roll threshold for scorable CELL ─────────────────────────────────────
    private static final double SCORABLE_ROLL_THRESHOLD_DEG = 90.0;

    // ── Alliance color ────────────────────────────────────────────────────────
    public enum Alliance { RED, BLUE }

    // ── HiveTarget — result returned to callers ───────────────────────────────
    /**
     * All information needed to aim at and drive toward a HIVE CELL.
     * range and bearing are already smoothed via rolling average.
     * Obtain via {@link #getHiveTarget(Alliance)}.
     */
    public static class HiveTarget {
        /** Cluster name, e.g. "RED AUDIENCE". */
        public final String name;
        /** Smoothed distance to the CELL opening in inches. */
        public final double range;
        /**
         * Smoothed horizontal angle to the CELL opening in degrees.
         * Positive = CELL is to the robot's LEFT; negative = RIGHT.
         * Steer until this is close to 0° to be aimed straight at the CELL.
         */
        public final double bearing;
        /** Fraction of the cluster's 4 tags that were detected (0–100). */
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

    // ── Rolling average buffer ────────────────────────────────────────────────
    /** Keeps a fixed-size window of recent values and returns their mean. */
    private static class RollingAverage {
        private final int             size;
        private final ArrayDeque<Double> bearings = new ArrayDeque<>();
        private final ArrayDeque<Double> ranges   = new ArrayDeque<>();

        RollingAverage(int size) { this.size = size; }

        void add(double bearing, double range) {
            if (bearings.size() >= size) { bearings.pollFirst(); ranges.pollFirst(); }
            bearings.addLast(bearing);
            ranges.addLast(range);
        }

        void clear() { bearings.clear(); ranges.clear(); }

        boolean hasData() { return !bearings.isEmpty(); }

        double avgBearing() {
            double sum = 0; for (double v : bearings) sum += v; return sum / bearings.size();
        }

        double avgRange() {
            double sum = 0; for (double v : ranges) sum += v; return sum / ranges.size();
        }
    }

    // ─────────────────────────────────────────────────────────────────────────

    private final VisionPortal      visionPortal;
    private final AprilTagProcessor aprilTag;

    // Rolling average buffers — one per alliance.
    private final RollingAverage avgRed  = new RollingAverage(ROLLING_AVERAGE_SIZE);
    private final RollingAverage avgBlue = new RollingAverage(ROLLING_AVERAGE_SIZE);

    // CPU management state.
    private boolean processorEnabled = true;
    private boolean streaming        = true;

    public Vision(HardwareMap hardwareMap) {
        RobotLog.ii(TAG, "Initialising — camera: \"%s\"  resolution: %dx%d",
                CAMERA_NAME, FRAME_WIDTH, FRAME_HEIGHT);

        if (hardwareMap.tryGet(WebcamName.class, CAMERA_NAME) == null) {
            RobotLog.ee(TAG, "Camera \"%s\" NOT found in hardware config. " +
                    "Check the Driver Station configuration matches CAMERA_NAME.", CAMERA_NAME);
        }

        // Build the AprilTag processor with lens calibration.
        AprilTagProcessor.Builder aprilTagBuilder = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setDrawTagID(true);

        if (USE_CALIBRATION) {
            aprilTagBuilder.setLensIntrinsics(CAL_FX, CAL_FY, CAL_CX, CAL_CY);
            RobotLog.ii(TAG, "Lens calibration applied: fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
                    CAL_FX, CAL_FY, CAL_CX, CAL_CY);
        }

        aprilTag = aprilTagBuilder.build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .addProcessor(aprilTag)
                .setCameraResolution(new android.util.Size(FRAME_WIDTH, FRAME_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStartStreamOnBuild(false)  // don't stream until explicitly requested
                .enableLiveView(false)             // no live view — saves CPU and memory
                .setShowStatsOverlay(false)        // no stats overlay
                .build();

        // Start with processor disabled — enable only when actively targeting.
        visionPortal.setProcessorEnabled(aprilTag, false);
        processorEnabled = false;
        streaming = false;

        RobotLog.ii(TAG, "VisionPortal built — stream and processor both OFF. " +
                "Call resumeStream() + enable() before reading targets.");
    }

    // ── Subsystem ─────────────────────────────────────────────────────────────

    @Override
    public void update() {
        // VisionPortal runs detection on its own background thread.
        // Nothing to poll here.
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

                long ageMs = (System.nanoTime() - cd.frameAcquisitionNanoTime) / 1_000_000L;
                boolean fresh    = ageMs <= MAX_DETECTION_AGE_MS;
                boolean scorable = isScorableCell(cd);
                boolean confident = cd.percentClusterFound >= MIN_CLUSTER_PERCENT;

                telemetry.addData(
                        "Vision/" + cd.metadata.name,
                        "range=%.1f\"  bear=%.1f°  roll=%.1f°  tags=%d%%  age=%dms  %s",
                        cd.ftcPose.range,
                        cd.ftcPose.bearing,
                        cd.ftcPose.roll,
                        cd.percentClusterFound,
                        ageMs,
                        (!fresh ? "STALE" : !confident ? "low-confidence" :
                                scorable ? "SCORABLE" : "not scorable"));
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
     * Starts the stream if not already running, then blocks until the camera
     * reaches STREAMING state — up to 15 seconds. Safe to call even if the
     * stream is already running.
     */
    public void waitUntilReady() {
        waitUntilReady(null);
    }

    /**
     * Same as {@link #waitUntilReady()} but also pushes state updates to the
     * Driver Station telemetry so you can see progress without needing Logcat.
     */
    public void waitUntilReady(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        // Start the stream if it isn't already running.
        if (!streaming) {
            RobotLog.ii(TAG, "waitUntilReady: starting stream.");
            visionPortal.resumeStreaming();
            streaming = true;
        }

        long deadline  = System.currentTimeMillis() + 15_000;
        long lastLogMs = System.currentTimeMillis();

        RobotLog.ii(TAG, "Waiting for camera \"%s\" to reach STREAMING state...", CAMERA_NAME);

        while (!isReady() && System.currentTimeMillis() < deadline) {
            long elapsed = 15_000 - (deadline - System.currentTimeMillis());

            if (System.currentTimeMillis() - lastLogMs >= 1000) {
                VisionPortal.CameraState state = visionPortal.getCameraState();
                RobotLog.ii(TAG, "Camera state: %s  (%.1f s elapsed)", state, elapsed / 1000.0);

                if (telemetry != null) {
                    telemetry.addData("Camera", state.toString());
                    telemetry.addData("Waiting", "%.1f s / 15.0 s", elapsed / 1000.0);
                    telemetry.update();
                }
                lastLogMs = System.currentTimeMillis();
            }

            try { Thread.sleep(100); } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                RobotLog.ww(TAG, e, "waitUntilReady interrupted before camera was ready.");
                break;
            }
        }

        if (isReady()) {
            RobotLog.ii(TAG, "Camera is STREAMING and ready.");
        } else {
            streaming = false; // reset — stream never actually started
            RobotLog.ee(TAG, "Camera did NOT reach STREAMING state within 15 seconds. " +
                    "Final state: %s. " +
                    "Check: (1) camera in USB 3.0 (blue) port, " +
                    "(2) camera named \"%s\" in Driver Station config, " +
                    "(3) try unplugging and replugging the camera.",
                    visionPortal.getCameraState(), CAMERA_NAME);
        }
    }

    // ── CPU management ────────────────────────────────────────────────────────

    /** Enables the AprilTag processor. Instant — camera stays streaming. */
    public void enable() {
        if (!processorEnabled) {
            visionPortal.setProcessorEnabled(aprilTag, true);
            processorEnabled = true;
            RobotLog.ii(TAG, "AprilTag processor ENABLED.");
        }
    }

    /** Disables the AprilTag processor. Clears rolling averages. */
    public void disable() {
        if (processorEnabled) {
            visionPortal.setProcessorEnabled(aprilTag, false);
            processorEnabled = false;
            avgRed.clear();
            avgBlue.clear();
            RobotLog.ii(TAG, "AprilTag processor DISABLED. Rolling averages cleared.");
        }
    }

    /** True if the AprilTag processor is currently enabled. */
    public boolean isEnabled() { return processorEnabled; }

    /**
     * Fully stops the camera stream. Zero CPU cost.
     * Call resumeStream() ~500 ms before you need a detection.
     */
    public void stopStream() {
        if (streaming) {
            RobotLog.ii(TAG, "Stopping camera stream.");
            visionPortal.stopStreaming();
            streaming = false;
            avgRed.clear();
            avgBlue.clear();
        }
    }

    /**
     * Resumes the camera stream after stopStream(). Blocks until streaming
     * or 5 second timeout. Clears rolling averages on resume.
     */
    public void resumeStream() {
        if (!streaming) {
            RobotLog.ii(TAG, "Resuming camera stream...");
            visionPortal.resumeStreaming();
            streaming = true;

            long deadline = System.currentTimeMillis() + 5000;
            while (!isReady() && System.currentTimeMillis() < deadline) {
                try { Thread.sleep(30); } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    RobotLog.ww(TAG, e, "resumeStream interrupted while waiting for STREAMING state.");
                    break;
                }
            }

            if (isReady()) {
                RobotLog.ii(TAG, "Camera stream RESUMED.");
            } else {
                RobotLog.ee(TAG, "Camera did NOT return to STREAMING after resumeStream(). " +
                        "Current state: %s.", visionPortal.getCameraState());
            }
        }
    }

    /** True if the camera stream is currently running. */
    public boolean isStreaming() { return streaming; }

    // ── HIVE targeting ────────────────────────────────────────────────────────

    /**
     * Returns a {@link HiveTarget} for the best scorable HIVE CELL for the given alliance.
     *
     * Applies all accuracy filters:
     *   - confidence filter (percentClusterFound >= MIN_CLUSTER_PERCENT)
     *   - age check (frameAcquisitionNanoTime within MAX_DETECTION_AGE_MS)
     *   - scorable check (|roll| < 90°)
     *   - rolling average smoothing on range and bearing
     *
     * Returns {@code null} if no valid target is visible or vision is inactive.
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
        RollingAverage avg = (alliance == Alliance.RED) ? avgRed : avgBlue;

        if (detections == null || detections.isEmpty()) {
            avg.clear(); // lost sight — reset average so stale data doesn't linger
            return null;
        }

        String prefix = (alliance == Alliance.RED) ? "R" : "B";
        AprilTagClusterDetection bestRaw = null;

        for (AprilTagDetection d : detections) {
            if (!(d instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection cd = (AprilTagClusterDetection) d;

            // Filter: alliance color.
            if (!cd.metadata.name.startsWith(prefix)) continue;

            // Filter: scorable cell (right-side up).
            if (!isScorableCell(cd)) continue;

            // Accuracy improvement 2 — confidence filter.
            if (cd.percentClusterFound < MIN_CLUSTER_PERCENT) {
                RobotLog.dd(TAG, "Rejected %s — only %d%% of cluster tags found (min %d%%).",
                        cd.metadata.name, cd.percentClusterFound, MIN_CLUSTER_PERCENT);
                continue;
            }

            // Accuracy improvement 4 — age check.
            long ageMs = (System.nanoTime() - cd.frameAcquisitionNanoTime) / 1_000_000L;
            if (ageMs > MAX_DETECTION_AGE_MS) {
                RobotLog.dd(TAG, "Rejected %s — detection is %d ms old (max %d ms).",
                        cd.metadata.name, ageMs, MAX_DETECTION_AGE_MS);
                continue;
            }

            // Pick the closest valid target.
            if (bestRaw == null || cd.ftcPose.range < bestRaw.ftcPose.range) {
                bestRaw = cd;
            }
        }

        if (bestRaw == null) {
            avg.clear(); // no valid target — reset average
            return null;
        }

        // Accuracy improvement 3 — feed into rolling average.
        avg.add(bestRaw.ftcPose.bearing, bestRaw.ftcPose.range);

        HiveTarget target = new HiveTarget(
                bestRaw.metadata.name,
                avg.avgRange(),
                avg.avgBearing(),
                bestRaw.percentClusterFound);

        RobotLog.dd(TAG, "getHiveTarget(%s): %s", alliance, target);
        return target;
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

    /** A CELL is scorable when its cluster is right-side up: |roll| < 90°. */
    private static boolean isScorableCell(AprilTagClusterDetection cd) {
        return Math.abs(cd.ftcPose.roll) < SCORABLE_ROLL_THRESHOLD_DEG;
    }
}
