package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.Tests;

/*
 * Tuning procedures, run from the Pedro tuning web page on the robot.
 * Order: Mecanum -> Pinpoint -> Foresight -> Tests. Paste each result into Constants.
 */
public class Tuning {

    @Tuner(name = "Mecanum")
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "Pinpoint")
    public static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner(name = "Foresight")
    public static Procedure foresight() {
        return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
    }

    @Tuner(name = "Tests")
    public static Procedure tests() {
        return new Tests(Constants::createDrivetrain, Constants::createLocalizer, Constants::createAlgorithm);
    }
}
