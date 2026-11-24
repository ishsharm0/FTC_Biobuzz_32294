package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Front intake. Optional: does nothing when the config has no "intake" motor. */
public class Intake implements Subsystem {

    public static double POWER = 0.5;

    private final DcMotor motor;
    private double power;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.tryGet(DcMotor.class, "intake");
        if (motor != null) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /** 1 pulls balls in, -1 spits them out, 0 stops. */
    public void set(int direction) {
        power = direction * POWER;
    }

    @Override
    public void update() {
        if (motor != null) {
            motor.setPower(power);
        }
    }

    @Override
    public void stop() {
        power = 0.0;
        update();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", motor == null ? "not in config" : power > 0 ? "in" : power < 0 ? "out" : "off");
    }
}
