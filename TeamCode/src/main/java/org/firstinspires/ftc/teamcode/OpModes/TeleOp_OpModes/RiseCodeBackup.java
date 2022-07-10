package org.firstinspires.ftc.teamcode.OpModes.TeleOp_OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Control.TeleOp.RiseTeleOp_FF_Control;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.BotMechanism;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.MecanumDriveTrain;

@Disabled
public class RiseCodeBackup extends LinearOpMode {
    @Override
    public void runOpMode() {
        RiseTeleOp_FF_Control chassis = new RiseTeleOp_FF_Control("frontLeft", "frontRight", "backRight", "backLeft", "armLeft", "armRight", "cageSpin", "gate", "intake",hardwareMap, telemetry, gamepad1);
        BotMechanism mechanisms = new BotMechanism("intake", "armLeft", "armRight",hardwareMap, telemetry, gamepad1);

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){
            chassis.teleOpDrive();
            mechanisms.Intake();
            mechanisms.Arm();
            mechanisms.toggleGate();
            mechanisms.Telemetry();
        }
    }
}
