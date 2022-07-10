package org.firstinspires.ftc.teamcode.OpModes.TeleOp_OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.TeleOp.RiseTeleOp_FF_Control;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.BotMechanism;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.MecanumDriveTrain;

@TeleOp
public class RiseTeleOp_FF extends LinearOpMode {

    @Override
    public void runOpMode() {

        //TODO: Make chart for classes
        RiseTeleOp_FF_Control teleOp = new RiseTeleOp_FF_Control("frontLeft", "frontRight", "backRight", "backLeft", "armLeft",  "armRight", "cage", "gate", "intake", hardwareMap, telemetry, gamepad1);
        BotMechanism mechanisms = new BotMechanism("intake", "armLeft", "armRight", hardwareMap, telemetry, gamepad1);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            teleOp.teleOpDrive();
            teleOp.teleOpArm();
            teleOp.teleOpGate();
            teleOp.teleOpIntake();

//            mechanisms.Intake();
//            mechanisms.Arm();
//            mechanisms.cageRotation();
//            mechanisms.toggleGate();
//            mechanisms.Telemetry();
        }
    }
}