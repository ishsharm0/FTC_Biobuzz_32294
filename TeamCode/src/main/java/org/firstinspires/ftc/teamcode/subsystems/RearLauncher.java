package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Rear outtake, fed by the "transfer" motor. Optional: autos skip shooting without it.
 *
 * TeleOp holds a button to feed continuously. Auto fires timed pulses: there is no ball
 * sensor, so tune FEED_SECONDS on the robot until one pulse fires exactly one ball.
 */
public class RearLauncher implements Subsystem {

    public static double POWER = 1.0;
    public static double FEED_SECONDS = 0.25;
    // Time from one shot to the next, including the feed pulse.
    public static double SHOT_INTERVAL_SECONDS = 0.9;

    private final DcMotor motor;
    private final ElapsedTime sinceShot = new ElapsedTime();
    private boolean pulsing;
    private boolean feeding;

    public RearLauncher(HardwareMap hardwareMap) {
        motor = hardwareMap.tryGet(DcMotor.class, "transfer");
        if (motor != null) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public boolean available() {
        return motor != null;
    }

    /** TeleOp: run the feed while true. */
    public void feed(boolean on) {
        feeding = on;
    }

    /** Auto: fire one timed pulse. Returns false, firing nothing, until the last shot has recovered. */
    public boolean shootOne() {
        if (!available() || (pulsing && sinceShot.seconds() < SHOT_INTERVAL_SECONDS)) {
            return false;
        }
        pulsing = true;
        sinceShot.reset();
        update();
        return true;
    }

    private boolean pulseFeeding() {
        return pulsing && sinceShot.seconds() < FEED_SECONDS;
    }

    @Override
    public void update() {
        if (motor != null) {
            motor.setPower(feeding || pulseFeeding() ? POWER : 0.0);
        }
    }

    @Override
    public void stop() {
        feeding = pulsing = false;
        update();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Launcher", motor == null ? "no transfer motor in config"
                : feeding || pulseFeeding() ? "feeding" : "idle");
    }
}
