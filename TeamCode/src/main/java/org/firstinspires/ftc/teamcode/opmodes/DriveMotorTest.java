package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Drive motor test — Control Hub ports 0-3.
 *
 * Select which motor to test with the D-pad (or bumpers), then spin it
 * with the triggers. This deliberately bypasses Pedro Pathing so you are
 * talking directly to the motor controllers.
 *
 *   D-pad UP / RIGHT bumper  → next motor
 *   D-pad DOWN / LEFT bumper → previous motor
 *   Left trigger  (hold)     → spin REVERSE  (power = trigger depth)
 *   Right trigger (hold)     → spin FORWARD  (power = trigger depth)
 *   Both triggers released   → motor stops
 *
 * Motors are run RAW (RUN_WITHOUT_ENCODER, no direction inversion) so you
 * can observe exactly what the hardware is doing and confirm wiring.
 */
@TeleOp(name = "Drive Motor Test (TEMP)", group = "BIOBUZZ Test")
public class DriveMotorTest extends LinearOpMode {

    // Port order matches the Control Hub physical ports 0-3.
    private static final String[] NAMES = {
            "motor0",      // port 0
            "backLeft",    // port 1
            "frontRight",  // port 2
            "backRight"    // port 3
    };

    @Override
    public void runOpMode() {

        // Grab all four motors raw — no direction flips so we see the true output.
        DcMotor[] motors = new DcMotor[NAMES.length];
        for (int i = 0; i < NAMES.length; i++) {
            motors[i] = hardwareMap.get(DcMotor.class, NAMES[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setDirection(DcMotorSimple.Direction.FORWARD);
        }

        int selected = 0;

        // D-pad / bumper debounce
        boolean lastUp    = false;
        boolean lastDown  = false;
        boolean lastRB    = false;
        boolean lastLB    = false;

        telemetry.addLine("Drive Motor Test ready");
        telemetry.addLine("D-pad UP/DOWN or bumpers to pick motor");
        telemetry.addLine("RT = forward  |  LT = reverse");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ── Motor selection ──────────────────────────────────────────────
            boolean up   = gamepad1.dpad_up   || gamepad1.right_bumper;
            boolean down = gamepad1.dpad_down  || gamepad1.left_bumper;

            if (up && !lastUp && !lastRB) {
                selected = (selected + 1) % NAMES.length;
            }
            if (down && !lastDown && !lastLB) {
                selected = (selected + NAMES.length - 1) % NAMES.length;
            }

            lastUp   = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastRB   = gamepad1.right_bumper;
            lastLB   = gamepad1.left_bumper;

            // ── Power ────────────────────────────────────────────────────────
            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            // Apply a small dead-zone so a resting trigger doesn't creep.
            if (Math.abs(power) < 0.05) power = 0.0;

            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(i == selected ? power : 0.0);
            }

            // ── Telemetry ────────────────────────────────────────────────────
            telemetry.addLine("=== DRIVE MOTOR TEST ===");
            telemetry.addLine("");
            for (int i = 0; i < NAMES.length; i++) {
                String arrow  = (i == selected) ? " ◄" : "";
                String status = (i == selected) ? String.format("%.2f", power) : "idle";
                telemetry.addData(
                        String.format("Port %d  %-11s", i, NAMES[i]),
                        "%s%s", status, arrow
                );
            }
            telemetry.addLine("");
            telemetry.addData("Encoder (selected)", motors[selected].getCurrentPosition());
            telemetry.addLine("");
            telemetry.addLine("RT=fwd  LT=rev  dpad/bumpers=select");
            telemetry.update();
        }

        // Zero all motors on stop.
        for (DcMotor m : motors) {
            m.setPower(0.0);
        }
    }
}
