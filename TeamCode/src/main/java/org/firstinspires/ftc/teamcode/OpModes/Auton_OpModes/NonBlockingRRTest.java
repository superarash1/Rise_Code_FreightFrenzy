package org.firstinspires.ftc.teamcode.OpModes.Auton_OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Control.Autonomous.RoadRunnerTest_Control;

@Autonomous
public class NonBlockingRRTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        RoadRunnerTest_Control control = new RoadRunnerTest_Control("frontLeft", "frontRight", "backRight", "backLeft", hardwareMap, telemetry);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()){
            control.RR_Drive();

        }
    }
}
