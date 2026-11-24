package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Rear outtake driven by the existing transfer motor. Front intake stays off during preload shots.
 * This is timed feeding, not a ball counter: tune FEED_SECONDS so one pulse feeds exactly one ball.
 */
public final class RearLauncher implements Subsystem {
    public static String MOTOR_NAME = "transfer";
    public static double POWER = 1.0;
    // Initial estimate only. Measure one-ball feed time on the robot before a match.
    public static double FEED_SECONDS = 0.25;
    // Team estimate for a fully loaded rear shooter, 2026-11-10.
    public static double SHOT_INTERVAL_SECONDS = 0.9;

    private final DcMotor motor;
    private final ElapsedTime clock = new ElapsedTime();
    private final ShotCycle cycle = new ShotCycle();

    public RearLauncher(HardwareMap map) {
        motor = map.tryGet(DcMotor.class, MOTOR_NAME);
        if (motor != null) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0.0);
        }
    }

    public boolean available() { return motor != null; }

    /** Starts at most one feed pulse. A second request during recovery is refused. */
    public boolean shootOne() {
        boolean accepted = available() && cycle.start(clock.seconds(), FEED_SECONDS, SHOT_INTERVAL_SECONDS);
        update();
        return accepted;
    }

    @Override public void update() {
        if (motor != null) motor.setPower(cycle.feeding(clock.seconds()) ? POWER : 0.0);
    }

    @Override public void stop() {
        cycle.stop();
        if (motor != null) motor.setPower(0.0);
    }

    @Override public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Rear outtake", !available() ? "transfer motor missing: drive/park only"
                : cycle.feeding(clock.seconds()) ? "feeding one" : cycle.busy(clock.seconds()) ? "recovering" : "ready");
    }
}
