package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ArmTesting extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDriveTrain chassis = new MecanumDriveTrain("frontLeft", "frontRight", "backRight", "backLeft",hardwareMap, telemetry, gamepad1);
        BotMechanisms mechanisms = new BotMechanisms("intake", "armLeft", "armRight",hardwareMap, telemetry, gamepad1);

        telemetry.addLine("Waiting for start");
        waitForStart();

        while (opModeIsActive()){
            chassis.teleOpDrive();
            mechanisms.Arm();
        }
    }
}
