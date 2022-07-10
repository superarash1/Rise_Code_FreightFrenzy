package org.firstinspires.ftc.teamcode.OpModes.TeleOp_OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.TeleOp.RiseTeleOp_FF_Control;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.BotMechanism;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.MecanumDriveTrain;

@TeleOp
public class LocalizationTesting extends LinearOpMode {

    @Override
    public void runOpMode() {
        RiseTeleOp_FF_Control teleOp = new RiseTeleOp_FF_Control("frontLeft", "frontRight", "backRight", "backLeft", "armLeft", "armRight", "cageSpin", "gate", "intake", hardwareMap, telemetry, gamepad1);
        waitForStart();

        while (opModeIsActive()) {
            teleOp.teleOpDrive();
        }
    }
}