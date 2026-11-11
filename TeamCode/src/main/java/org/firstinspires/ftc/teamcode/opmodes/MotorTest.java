package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * TEMPORARY motor test opmode — delete when done.
 *
 * Controls:
 *   Left trigger (hold)  → intake full power
 *   Right trigger (hold) → transfer full power
 */
@TeleOp(name = "Motor Test (TEMP)", group = "BIOBUZZ")
public class MotorTest extends LinearOpMode {

    public static double INTAKE_POWER  = 1.0;
    public static double TRANSFER_POWER = 0.747636685;

    @Override
    public void runOpMode() {
        DcMotor intake   = hardwareMap.get(DcMotor.class, "intake");
        DcMotor transfer = hardwareMap.get(DcMotor.class, "transfer");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE );

        for (DcMotor m : new DcMotor[]{intake, transfer}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Motor Test ready — waiting for start");
        telemetry.addLine("LT: intake  |  RT: transfer");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            intake.setPower(gamepad1.left_trigger > 0.1 ? INTAKE_POWER : 0.0);
            transfer.setPower(gamepad1.right_trigger > 0.1 ? TRANSFER_POWER : 0.0);

            telemetry.addData("Intake",   gamepad1.left_trigger > 0.1  ? "ON" : "off");
            telemetry.addData("Transfer", gamepad1.right_trigger > 0.1 ? "ON" : "off");
            telemetry.addData("Intake encoder",   intake.getCurrentPosition());
            telemetry.addData("Transfer encoder", transfer.getCurrentPosition());
            telemetry.update();
        }

        intake.setPower(0);
        transfer.setPower(0);
    }
}
