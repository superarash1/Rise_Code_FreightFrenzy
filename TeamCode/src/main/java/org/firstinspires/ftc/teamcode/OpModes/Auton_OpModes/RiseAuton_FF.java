package org.firstinspires.ftc.teamcode.OpModes.Auton_OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Control.Autonomous.TestAuton_Control;

@Autonomous
public class RiseAuton_FF extends LinearOpMode {

    @Override
    public void runOpMode() {

        TestAuton_Control auton = new TestAuton_Control("frontLeft", "frontRight", "backRight", "backLeft", hardwareMap, telemetry, gamepad1);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            auton.Drive();
            auton.Turret();
            auton.Telemetry();
            telemetry.update();
        }
    }
}
