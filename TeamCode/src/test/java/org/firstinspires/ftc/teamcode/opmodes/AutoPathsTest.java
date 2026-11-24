package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.Field;
import org.junit.Test;
import static org.junit.Assert.*;

public class AutoPathsTest {
    @Test public void sourceStartsTouchThePhysicalWallsAndRearFacesTheHive() {
        AutoPaths red = new AutoPaths(Field.Alliance.RED);
        assertEquals(Field.WALL_INSET, red.wallStart.x() - Field.ROBOT_LENGTH / 2, 1e-9);
        assertEquals(Field.WALL_INSET, red.audienceStart.y() - Field.ROBOT_LENGTH / 2, 1e-9);
        assertEquals(-1.0, Math.sin(red.launch.heading()), 1e-9);
        assertEquals(20.0, red.launch.y(), 1e-9);
        assertEquals(-0.3237, (red.launch.x() - 72) * 0.0254, 1e-9);
        assertTrue(red.wallStart.y() + Field.ROBOT_WIDTH / 2 < Field.LOADING_ZONE_Y_MIN);
    }

    @Test public void fullRobotFootprintsStayClearOfFrameWallsAndOpposingHalf() {
        AutoPaths red = new AutoPaths(Field.Alliance.RED);
        for (Path path : new Path[]{red.wallToPark, red.startToLaunch, red.launchToPark}) {
            for (int i = 0; i <= 1000; i++) {
                Pose p = path.get(i / 1000.0);
                double halfX = Math.abs(Math.cos(p.heading())) * Field.ROBOT_LENGTH / 2
                        + Math.abs(Math.sin(p.heading())) * Field.ROBOT_WIDTH / 2;
                double halfY = Math.abs(Math.sin(p.heading())) * Field.ROBOT_LENGTH / 2
                        + Math.abs(Math.cos(p.heading())) * Field.ROBOT_WIDTH / 2;
                assertTrue(p.x() - halfX >= Field.WALL_INSET - 1e-6);
                assertTrue(p.y() - halfY >= Field.WALL_INSET - 1e-6);
                assertTrue(p.y() + halfY <= Field.SIZE - Field.WALL_INSET);
                assertTrue(p.x() + halfX < Field.SIZE / 2);
                boolean clear = p.x() + halfX < Field.FRAME_X_MIN || p.x() - halfX > Field.FRAME_X_MAX
                        || p.y() + halfY < Field.FRAME_Y_MIN || p.y() - halfY > Field.FRAME_Y_MAX;
                assertTrue("Frame collision at path fraction " + i, clear);
            }
        }
    }

    @Test public void bluePathsAreAHalfTurnOfRed() {
        AutoPaths red = new AutoPaths(Field.Alliance.RED), blue = new AutoPaths(Field.Alliance.BLUE);
        for (int i = 0; i <= 100; i++) {
            Pose r = red.launchToPark.get(i / 100.0), b = blue.launchToPark.get(i / 100.0);
            assertEquals(Field.SIZE - r.x(), b.x(), 1e-8);
            assertEquals(Field.SIZE - r.y(), b.y(), 1e-8);
            assertEquals(-1.0, Math.cos(b.heading() - r.heading()), 1e-8);
        }
    }
}
