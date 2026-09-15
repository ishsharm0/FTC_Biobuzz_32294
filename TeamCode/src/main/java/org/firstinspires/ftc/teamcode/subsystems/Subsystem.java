package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** A mechanism on the robot. Robot calls update() once per loop on every registered subsystem. */
public interface Subsystem {

    void update();

    default void stop() {}

    default void addTelemetry(Telemetry telemetry) {}
}
