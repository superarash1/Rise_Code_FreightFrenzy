package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Autonomous.TestAuton_Control;

public class BotTelemetry {
    private Telemetry telemetry;

    private HardwareMap hardwareMap;
    private Gamepad gamepad1;
//
//    Cage cage = new Cage("cageSpin", hardwareMap, gamepad1);
//    Intake intake = new Intake("intake", hardwareMap, gamepad1);
//
//    Arm arm = new Arm("arnLeft", "armRight",hardwareMap, gamepad1);

//    MecanumDriveTrain chassis = new MecanumDriveTrain("frontLeft", "frontRight", "backRight", "backLeft", hardwareMap, telemetry, gamepad1);

    TestAuton_Control auton;


    public BotTelemetry(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1){
        auton =  new TestAuton_Control("frontLeft", "frontRight", "backRight", "backLeft", hardwareMap, telemetry, gamepad1);

        this.telemetry = telemetry;

        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void Telemetry(){
//        telemetry.addData("Cage Spin Position:", cage.cageSpin.getPosition());
//        telemetry.addData("intake position", intake.getPosTicks());

//        chassis.outputEncoderReadings();
//
//        telemetry.addData("Angle", arm.armLeft.getCurrPosDegrees());
//        telemetry.addData("Target Position", arm.targetPos);

        telemetry.addData("Auton Step", auton.driveStep);


        telemetry.update();
    }
}
