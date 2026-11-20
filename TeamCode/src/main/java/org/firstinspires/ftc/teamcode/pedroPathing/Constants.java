package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * Pedro Pathing 3 drivetrain + localizer + Foresight config.
 *
 * ── Tuning status ────────────────────────────────────────────────────────────
 * Drivetrain (MecanumTuner)  : TUNED — motor names and directions confirmed.
 * Localizer  (PinpointTuner) : TUNED — pod offsets carried over from DECODE 2.x.
 * Foresight  (ForesightTuner): PARTIALLY TUNED
 *     maxAchievableForwardVelocity  ✓ measured on DECODE robot
 *     maxAchievableStrafeVelocity   ✓ measured on DECODE robot
 *     naturalForwardDeceleration    ✓ measured on DECODE robot
 *     naturalStrafeDeceleration     ✓ measured on DECODE robot
 *     All controllers               ✗ PLACEHOLDER — run ForesightTuner and
 *                                     paste the generated foresightConfig here.
 *
 * ── How to re-tune ───────────────────────────────────────────────────────────
 * 1. Connect to the robot's Wi-Fi, deploy, open http://192.168.43.1:8080/tuning
 * 2. Run: Mecanum → Pinpoint → Foresight → Tests (in that order)
 * 3. Paste each result into the matching config block below.
 */
public class Constants {

    // ── Drivetrain ────────────────────────────────────────────────────────────
    // Source: MecanumTuner — confirmed on BIOBUZZ robot.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.backLeftName.set("backLeft");
        c.frontRightName.set("frontRight");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // ── Localizer ─────────────────────────────────────────────────────────────
    // Source: PinpointTuner — pod offsets carried over from DECODE 2.x tuning.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.5);
        c.yPodOffset.set(-6.25);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // ── Foresight ─────────────────────────────────────────────────────────────
    // Velocity and deceleration values: measured on DECODE robot, carry over.
    // Controllers: PLACEHOLDER — replace with ForesightTuner output.
    //
    // The piecewise translational controllers use two kP zones:
    //   distance > 2.5 in  → primary (larger kP, used when far from target)
    //   distance <= 2.5 in → secondary (smaller kP, used for final approach)
    // Run ForesightTuner to find the correct values for this robot.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {

        // ── Translational controllers (PLACEHOLDER — run ForesightTuner) ──────
        Controller primaryTranslationalForward   = Controller.proportional(0.1);
        Controller secondaryTranslationalForward = Controller.proportional(0.05);
        Controller primaryTranslationalLateral   = Controller.proportional(0.1);
        Controller secondaryTranslationalLateral = Controller.proportional(0.05);

        c.forwardTranslational.set(
                Controller.piecewise(secondaryTranslationalForward)
                        .put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(
                Controller.piecewise(secondaryTranslationalLateral)
                        .put(2.5, primaryTranslationalLateral));

        // ── Feedforward (PLACEHOLDER — run ForesightTuner) ───────────────────
        c.coast.set(Controller.proportionalFeedforward(0.0));
        c.brake.set(Controller.proportionalFeedforward(0.0));

        // ── Heading (PLACEHOLDER — run ForesightTuner) ───────────────────────
        c.headingFeedback.set(Controller.proportional(1.0));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.1, 0.0));

        // ── Braking coefficients (PLACEHOLDER — run ForesightTuner) ─────────
        // Estimated as v² / (2 * deceleration); will be refined by tuner.
        c.linearBrakeCoefficients.set(Matrix.diag(0.05, 0.05));
        c.quadraticBrakeCoefficients.set(
                Matrix.diag(1.0 / (2.0 * 34.5977), 1.0 / (2.0 * 56.0205)));

        // ── Velocity / deceleration (measured on DECODE robot) ───────────────
        c.maxAchievableForwardVelocity.set(62.8303);
        c.maxAchievableStrafeVelocity.set(54.7012);
        c.naturalForwardDeceleration.set(34.5977);
        c.naturalStrafeDeceleration.set(56.0205);
    });

    // ── Factory methods ───────────────────────────────────────────────────────

    public static Mecanum createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static PinpointLocalizer createLocalizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    public static Foresight createAlgorithm() {
        return new Foresight(foresightConfig);
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(createLocalizer(hardwareMap), createDrivetrain(hardwareMap), createAlgorithm());
    }

    public static Follower create(HardwareMap hardwareMap) {
        return createFollower(hardwareMap);
    }
}
