package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/* LEAVE + PARK: off the alliance wall and into the loading zone. 8 points. */
@Autonomous(name = "Park Auto", group = "BIOBUZZ", preselectTeleOp = "Main TeleOp")
public class ParkAuto extends BiobuzzAuto {

    @Override
    protected Pose startPose() {
        return paths.wallStart;
    }

    @Override
    protected String placement() {
        return "back on the ALLIANCE wall, beside the loading zone on its audience side";
    }

    @Override
    protected void run() {
        if (step == 0) {
            follower.follow(paths.wallToPark);
            next();
        }
        // The follower holds the end pose once the path is done.
    }
}
