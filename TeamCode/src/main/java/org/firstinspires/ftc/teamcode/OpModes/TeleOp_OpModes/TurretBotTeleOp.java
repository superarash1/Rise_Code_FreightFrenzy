package org.firstinspires.ftc.teamcode.OpModes.TeleOp_OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.TeleOp.TurretBotTeleOp_Control;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.MecanumDriveTrain;

@TeleOp
public class TurretBotTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        TurretBotTeleOp_Control turret = new TurretBotTeleOp_Control("frontLeft", "frontRight", "backRight", "backLeft","spinny", hardwareMap, gamepad1, telemetry);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            turret.teleOpDrive();
            turret.turretSpin();
        }
        turret.closeCamera();
    }
}
