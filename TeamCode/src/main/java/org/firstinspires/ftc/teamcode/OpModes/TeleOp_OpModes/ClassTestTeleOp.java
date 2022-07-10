package org.firstinspires.ftc.teamcode.OpModes.TeleOp_OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.TeleOp.RiseTeleOp_FF_Control;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.Arm;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.Cage;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.Gate;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.Intake;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.BotTelemetry;

@TeleOp
public class ClassTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        RiseTeleOp_FF_Control teleOp = new RiseTeleOp_FF_Control("frontLeft", "frontRight", "backRight", "backLeft", "armLeft", "armRight", "cageSpin", "gate", "intake",hardwareMap, telemetry, gamepad1);
        Intake intake = new Intake("intake", hardwareMap);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            teleOp.teleOpDrive();
            teleOp.teleOpArm();
            teleOp.teleOpGate();
            teleOp.teleOpIntake();
        }
    }
}
