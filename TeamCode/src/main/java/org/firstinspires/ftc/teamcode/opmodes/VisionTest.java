package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Field;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/*
 * Camera check without the drivetrain. Shows every HIVE cluster the camera sees and the
 * target Main TeleOp would pick. X / B picks blue / red.
 * Range should match a tape measure; bearing should read about 0 when aimed at the cell.
 */
@TeleOp(name = "Vision Test", group = "BIOBUZZ")
public class VisionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap);
        Field.Alliance alliance = Field.Alliance.RED;

        waitForStart();
        vision.setActive(true);

        while (opModeIsActive()) {
            if (gamepad1.xWasPressed()) {
                alliance = Field.Alliance.BLUE;
            }
            if (gamepad1.bWasPressed()) {
                alliance = Field.Alliance.RED;
            }

            Vision.Target target = vision.target(alliance);
            telemetry.addData("Alliance (X blue, B red)", alliance);
            telemetry.addData("Target", target == null ? "none"
                    : String.format("%s  %.1f in  %.1f°  %s", target.name, target.range, target.bearing,
                            target.inScoringRange() ? "IN RANGE" : "out of range"));
            vision.addTelemetry(telemetry);
            telemetry.update();
        }
        vision.stop();
    }
}
