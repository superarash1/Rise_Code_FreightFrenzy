package org.firstinspires.ftc.teamcode.OpModes.Auton_OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Control.Autonomous.TestAuton_Control;
import org.firstinspires.ftc.teamcode.Control.Autonomous.setVelocityTesting_Control;

@Autonomous
public class setVelocityTesting extends LinearOpMode {
    @Override
    public void runOpMode() {

        setVelocityTesting_Control auton = new setVelocityTesting_Control("frontLeft", "frontRight", "backRight", "backLeft", hardwareMap, telemetry, gamepad1);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            auton.Drive();
            auton.Telemetry();
            telemetry.update();
        }
    }
}
