package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.MecanumDriveTrain;

public class DriveEncoderOdometry {

    MecanumDriveTrain driveTrain;
    Telemetry telemetry;

    public DriveEncoderOdometry(String flName, String frName, String brName, String blName, HardwareMap hardwareMap, Telemetry telemetry){

        driveTrain = new MecanumDriveTrain(flName, frName, brName, blName, hardwareMap);

        this.telemetry = telemetry;
    }

    public void outputEncoderReadings(){
        telemetry.addData("FrontLeft Encoder Position Inches : ", driveTrain.frontLeft.getCurrPosInches());
        telemetry.addData("FrontRight Encoder Position Inches : ", driveTrain.frontRight.getCurrPosInches());
        telemetry.addData("BackRight Encoder Position Inches : ", driveTrain.backRight.getCurrPosInches());
        telemetry.addData("BackLeft Encoder Position Inches : ", driveTrain.backLeft.getCurrPosInches());
    }
}
