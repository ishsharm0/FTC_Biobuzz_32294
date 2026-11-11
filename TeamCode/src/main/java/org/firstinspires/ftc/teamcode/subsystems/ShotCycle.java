package org.firstinspires.ftc.teamcode.subsystems;

/** A single feeder pulse with recovery time. The caller supplies a monotonic clock in seconds. */
public final class ShotCycle {
    private double feedUntil = Double.NEGATIVE_INFINITY;
    private double readyAt = Double.NEGATIVE_INFINITY;

    public boolean start(double now, double feedSeconds, double intervalSeconds) {
        if (!Double.isFinite(now) || !Double.isFinite(feedSeconds) || !Double.isFinite(intervalSeconds)
                || feedSeconds <= 0 || intervalSeconds < feedSeconds || busy(now)) {
            return false;
        }
        feedUntil = now + feedSeconds;
        readyAt = now + intervalSeconds;
        return true;
    }

    public boolean feeding(double now) { return now < feedUntil; }
    public boolean busy(double now) { return now < readyAt; }
    public void stop() { feedUntil = readyAt = Double.NEGATIVE_INFINITY; }
}
