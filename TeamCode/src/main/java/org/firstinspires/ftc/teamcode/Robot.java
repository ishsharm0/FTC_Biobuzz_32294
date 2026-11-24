package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.RearLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

/*
 * The whole robot. An OpMode makes one Robot and calls update() once per loop.
 * To add a mechanism, write a Subsystem and register() it in the constructor.
 */
public class Robot {

    // Set by the match autos so TeleOp aims at the right HIVE.
    public static Field.Alliance alliance = Field.Alliance.RED;

    public final Drivetrain drivetrain;
    public final Intake intake;
    public final RearLauncher launcher;
    /** Null when the config has no camera. */
    public final Vision vision;

    private final List<LynxModule> hubs;
    private final List<Subsystem> subsystems = new ArrayList<>();

    public Robot(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drivetrain = register(new Drivetrain(hardwareMap));
        intake = register(new Intake(hardwareMap));
        launcher = register(new RearLauncher(hardwareMap));
        vision = hardwareMap.tryGet(WebcamName.class, Vision.CAMERA_NAME) == null
                ? null
                : register(new Vision(hardwareMap));
    }

    private <T extends Subsystem> T register(T subsystem) {
        subsystems.add(subsystem);
        return subsystem;
    }

    public void update() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        for (Subsystem subsystem : subsystems) {
            subsystem.update();
        }
    }

    public void stop() {
        for (Subsystem subsystem : subsystems) {
            subsystem.stop();
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        for (Subsystem subsystem : subsystems) {
            subsystem.addTelemetry(telemetry);
        }
    }
}
