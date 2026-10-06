package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

/*
 * Hardware debugger. Doesn't touch Pedro or the Pinpoint, so it runs on a bare robot.
 * Put the robot on a stand (wheels off the ground) for the motor pages.
 *
 * gamepad1
 *   dpad left/right  switch page
 *   dpad up/down     select device
 *
 * MOTORS page: every motor in the config, one at a time.
 *   left stick Y     run the selected motor (up = positive power)
 *   B (hold)         run the selected motor at +TEST_POWER
 *   A                flip the selected motor's direction (only for this run)
 *   Y                reset encoder
 *
 * DRIVE CHECK page: uses the directions in pedroPathing/Constants.java.
 *   X (hold)         all four wheels at +TEST_POWER. Every wheel should roll the robot forward
 *                    and show a positive velocity. Any wheel marked FLIP needs its direction
 *                    swapped in Constants (and BasicTeleOp).
 *   dpad up/down     pick one wheel; B (hold) runs just that wheel forward
 *
 * SERVOS page: every servo in the config.
 *   left stick Y     nudge position
 *   A / B / X        go to 0 / 0.5 / 1
 */
@TeleOp(name = "DEBUG", group = "BIOBUZZ")
public class DebugTeleOp extends LinearOpMode {

    public static double TEST_POWER = 0.3;
    public static double SERVO_NUDGE_PER_LOOP = 0.005;

    private enum Page { MOTORS, DRIVE_CHECK, SERVOS }

    private static final String[] DRIVE_LABELS = {"front left", "back left", "front right", "back right"};

    @Override
    public void runOpMode() {
        List<DcMotorEx> motors = new ArrayList<>();
        List<String> motorNames = new ArrayList<>();
        for (DcMotorEx motor : hardwareMap.getAll(DcMotorEx.class)) {
            motors.add(motor);
            motorNames.add(nameOf(motor));
        }
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        List<Servo> servos = new ArrayList<>();
        List<String> servoNames = new ArrayList<>();
        for (Servo servo : hardwareMap.getAll(Servo.class)) {
            servos.add(servo);
            servoNames.add(nameOf(servo));
        }

        MecanumConfig config = Constants.drivetrainConfig;
        String[] driveNames = {
                config.frontLeftName.get(), config.backLeftName.get(),
                config.frontRightName.get(), config.backRightName.get()};
        DcMotorSimple.Direction[] driveDirections = {
                config.frontLeftDirection.get(), config.backLeftDirection.get(),
                config.frontRightDirection.get(), config.backRightDirection.get()};
        DcMotorEx[] drive = new DcMotorEx[4];
        for (int i = 0; i < 4; i++) {
            drive[i] = hardwareMap.tryGet(DcMotorEx.class, driveNames[i]);
        }

        Page page = Page.MOTORS;
        int motorIndex = 0, driveIndex = 0, servoIndex = 0;

        telemetry.addData("Motors", motorNames);
        telemetry.addData("Servos", servoNames);
        telemetry.addLine("Ready. Robot on a stand for motor tests.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // wasPressed() only reports true once per press, so read each one once.
            boolean next = gamepad1.dpadRightWasPressed();
            boolean prev = gamepad1.dpadLeftWasPressed();
            if (next || prev) {
                stopAll(motors);
                int step = next ? 1 : Page.values().length - 1;
                page = Page.values()[(page.ordinal() + step) % Page.values().length];
            }
            int select = gamepad1.dpadDownWasPressed() ? 1 : gamepad1.dpadUpWasPressed() ? -1 : 0;

            telemetry.addData("Page", "%s   (dpad L/R to switch)", page);

            switch (page) {
                case MOTORS: {
                    if (motors.isEmpty()) {
                        telemetry.addLine("No motors in the config.");
                        break;
                    }
                    if (select != 0) {
                        motors.get(motorIndex).setPower(0);
                        motorIndex = wrap(motorIndex + select, motors.size());
                    }
                    DcMotorEx motor = motors.get(motorIndex);
                    if (gamepad1.aWasPressed()) {
                        motor.setDirection(motor.getDirection() == DcMotorSimple.Direction.FORWARD
                                ? DcMotorSimple.Direction.REVERSE
                                : DcMotorSimple.Direction.FORWARD);
                    }
                    if (gamepad1.yWasPressed()) {
                        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                    double power = gamepad1.b ? TEST_POWER : -gamepad1.left_stick_y;
                    motor.setPower(power);

                    for (int i = 0; i < motors.size(); i++) {
                        DcMotorEx m = motors.get(i);
                        telemetry.addLine(String.format("%s %-12s %-7s pos %7d  vel %7.0f",
                                i == motorIndex ? ">" : " ", motorNames.get(i), m.getDirection(),
                                m.getCurrentPosition(), m.getVelocity()));
                    }
                    telemetry.addData("Power", "%.2f", power);
                    telemetry.addLine("Positive power should spin the mechanism its 'forward' way.");
                    telemetry.addLine("If not, press A and copy the new direction into the code.");
                    break;
                }

                case DRIVE_CHECK: {
                    if (select != 0) {
                        driveIndex = wrap(driveIndex + select, 4);
                    }
                    for (int i = 0; i < 4; i++) {
                        if (drive[i] == null) {
                            continue;
                        }
                        drive[i].setDirection(driveDirections[i]);
                        boolean run = gamepad1.x || (gamepad1.b && i == driveIndex);
                        drive[i].setPower(run ? TEST_POWER : 0);
                    }

                    for (int i = 0; i < 4; i++) {
                        String marker = i == driveIndex ? ">" : " ";
                        if (drive[i] == null) {
                            telemetry.addLine(String.format("%s %-11s '%s' MISSING from config",
                                    marker, DRIVE_LABELS[i], driveNames[i]));
                            continue;
                        }
                        double velocity = drive[i].getVelocity();
                        String verdict = drive[i].getPower() == 0 ? ""
                                : velocity > 20 ? "ok" : velocity < -20 ? "FLIP" : "not moving?";
                        telemetry.addLine(String.format("%s %-11s %-7s vel %7.0f  %s",
                                marker, DRIVE_LABELS[i], driveDirections[i], velocity, verdict));
                    }
                    telemetry.addLine("Hold X: all wheels forward. Hold B: selected wheel.");
                    telemetry.addLine("Watch each wheel too: it should roll the robot forward.");
                    telemetry.addLine("'not moving?' = unplugged motor or encoder cable.");
                    break;
                }

                case SERVOS: {
                    if (servos.isEmpty()) {
                        telemetry.addLine("No servos in the config.");
                        break;
                    }
                    if (select != 0) {
                        servoIndex = wrap(servoIndex + select, servos.size());
                    }
                    Servo servo = servos.get(servoIndex);
                    if (gamepad1.aWasPressed()) {
                        servo.setPosition(0);
                    } else if (gamepad1.bWasPressed()) {
                        servo.setPosition(0.5);
                    } else if (gamepad1.xWasPressed()) {
                        servo.setPosition(1);
                    } else if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                        double current = Double.isNaN(servo.getPosition()) ? 0.5 : servo.getPosition();
                        servo.setPosition(Math.max(0, Math.min(1,
                                current - gamepad1.left_stick_y * SERVO_NUDGE_PER_LOOP)));
                    }

                    for (int i = 0; i < servos.size(); i++) {
                        telemetry.addLine(String.format("%s %-12s pos %.3f",
                                i == servoIndex ? ">" : " ", servoNames.get(i), servos.get(i).getPosition()));
                    }
                    telemetry.addLine("Stick Y nudges. A/B/X = 0 / 0.5 / 1.");
                    break;
                }
            }

            telemetry.update();
        }

        stopAll(motors);
    }

    private String nameOf(HardwareDevice device) {
        return hardwareMap.getNamesOf(device).iterator().next();
    }

    private static int wrap(int index, int size) {
        return ((index % size) + size) % size;
    }

    private static void stopAll(List<DcMotorEx> motors) {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }
}
