package org.firstinspires.ftc.teamcode.OpModes.Auton_OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Control.Autonomous.BlockTrackAuton_Control;

@Autonomous
public class blockDetection extends LinearOpMode {

    @Override
    public void runOpMode() {

        BlockTrackAuton_Control auton = new BlockTrackAuton_Control("frontLeft", "frontRight", "backRight", "backLeft", "spinny",hardwareMap, telemetry, gamepad1);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            auton.Drive();
            auton.Telemetry();

            telemetry.update();
        }
        auton.closeCamera();
    }
}
