package org.firstinspires.ftc.teamcode;


import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

public class Testbot {

    // Where the robot was when the last OpMode ended, so TeleOp can continue from Auto.
    public static Pose lastPose = new Pose();

    public final Drivetrain drivetrain;

    private final List<LynxModule> hubs;
    private final List<Subsystem> subsystems = new ArrayList<>();

    public Testbot(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drivetrain = register(new Drivetrain(hardwareMap));
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
        lastPose = drivetrain.getPose();
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
