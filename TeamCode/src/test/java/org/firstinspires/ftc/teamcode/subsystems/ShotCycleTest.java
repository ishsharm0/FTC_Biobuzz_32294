package org.firstinspires.ftc.teamcode.subsystems;

import org.junit.Test;
import static org.junit.Assert.*;

public class ShotCycleTest {
    @Test public void feedsOnePulseAndRejectsRequestsDuringRecovery() {
        ShotCycle cycle = new ShotCycle();
        assertTrue(cycle.start(0.0, 0.25, 0.9));
        assertTrue(cycle.feeding(0.1));
        assertFalse(cycle.feeding(0.25));
        assertTrue(cycle.busy(0.89));
        assertFalse(cycle.start(0.89, 0.25, 0.9));
        assertTrue(cycle.start(0.9, 0.25, 0.9));
        assertFalse(cycle.start(1.79, 0.25, 0.9));
        assertTrue(cycle.start(1.8, 0.25, 0.9));
    }

    @Test public void stopCancelsAnActiveFeedAndRecovery() {
        ShotCycle cycle = new ShotCycle();
        cycle.start(0.0, 0.25, 0.9);
        cycle.stop();
        assertFalse(cycle.feeding(0.1));
        assertFalse(cycle.busy(0.1));
        assertTrue(cycle.start(0.1, 0.25, 0.9));
    }

    @Test public void invalidTimingNeverStartsTheMotor() {
        ShotCycle cycle = new ShotCycle();
        assertFalse(cycle.start(0.0, 1.0, 0.9));
        assertFalse(cycle.start(0.0, -0.1, 0.9));
        assertFalse(cycle.start(Double.NaN, 0.25, 0.9));
        assertFalse(cycle.feeding(0.0));
    }
}
