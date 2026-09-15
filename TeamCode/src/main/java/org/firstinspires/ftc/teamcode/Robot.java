package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

/*
 * The whole robot. OpModes create one Robot, call update() once per loop, and use
 * subsystems through its fields.
 *
 * Adding a mechanism: write a class that implements Subsystem, add a public final field
 * for it below, and construct it with register(...) in the constructor.
 */
public class Robot {

    // Where the robot was when the last OpMode ended, so TeleOp can continue from Auto.
    public static Pose lastPose = new Pose();

    public final Drivetrain drivetrain;

    private final List<LynxModule> hubs;
    private final List<Subsystem> subsystems = new ArrayList<>();

    public Robot(HardwareMap hardwareMap) {
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
