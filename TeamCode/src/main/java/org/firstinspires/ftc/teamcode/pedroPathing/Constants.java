package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
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
 * Pedro Pathing setup. Every OpMode that drives gets its motor names and directions from here.
 *
 * To tune: deploy, join the robot Wi-Fi, open http://192.168.43.1:10158 and run
 * Mecanum -> Pinpoint -> Foresight -> Tests, pasting each result below.
 * The Foresight controllers are still starting guesses; velocities and decelerations
 * were measured on last season's robot.
 */
public class Constants {

    // The front motors read backwards relative to the back ones on this robot (wiring or
    // gearbox), so both fronts are FORWARD and both backs are REVERSE. These directions
    // reproduce the Oct24 TeleOp that drove correctly on the robot. If one wheel spins the
    // wrong way, check it on the DEBUG OpMode's DRIVE CHECK page before changing anything here.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.backLeftName.set("backLeft");
        c.frontRightName.set("frontRight");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static final String PINPOINT_NAME = "pinpoint";

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(PINPOINT_NAME);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.5);
        c.yPodOffset.set(-6.25);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // Stronger correction far from the path, softer within 2.5 in.
        c.forwardTranslational.set(Controller.piecewise(Controller.proportional(0.05))
                .put(2.5, Controller.proportional(0.1)));
        c.strafeTranslational.set(Controller.piecewise(Controller.proportional(0.05))
                .put(2.5, Controller.proportional(0.1)));

        c.coast.set(Controller.proportionalFeedforward(0.0));
        c.brake.set(Controller.proportionalFeedforward(0.0));

        c.headingFeedback.set(Controller.proportional(1.0));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.1, 0.0));

        c.linearBrakeCoefficients.set(Matrix.diag(0.05, 0.05));
        c.quadraticBrakeCoefficients.set(Matrix.diag(1.0 / (2.0 * 34.5977), 1.0 / (2.0 * 56.0205)));

        c.maxAchievableForwardVelocity.set(62.8303);
        c.maxAchievableStrafeVelocity.set(54.7012);
        c.naturalForwardDeceleration.set(34.5977);
        c.naturalStrafeDeceleration.set(56.0205);
    });

    public static Mecanum createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static PinpointLocalizer createLocalizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    public static Foresight createAlgorithm() {
        return new Foresight(foresightConfig);
    }
}
