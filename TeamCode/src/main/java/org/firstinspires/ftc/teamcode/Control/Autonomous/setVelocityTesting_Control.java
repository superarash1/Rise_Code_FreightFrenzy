package org.firstinspires.ftc.teamcode.Control.Autonomous;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.Turret;

public class setVelocityTesting_Control {
    public MecanumDriveTrain chassis;
    public Turret turret;

    public Telemetry telemetry;
    public Gamepad gamepad1;
    public HardwareMap hardwareMap;

    public int driveStep = 1;

    public double power;

    public setVelocityTesting_Control(String flName, String frName, String brName, String blName, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1){
        this.chassis = new MecanumDriveTrain(flName, frName, brName, blName, hardwareMap);
        this.turret = new Turret("spinny", hardwareMap);
        this.chassis.reset();

        chassis.setBreakMode();

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
    }

    public void Drive(){
        if (driveStep == 1){

            chassis.setVelocity(-Math.PI);

            if (-chassis.frontLeft.getCurrPosInches() > 10){
                chassis.reset();

                // Make a nested method thingy so that I can just put one for all
                chassis.setVelocity(0);
                driveStep = 2;
            }
        }
//
//        if (driveStep == 2){
//            PID.PIDF(chassis.frontLeft.getCurrPosInches(), -15);
//
//            power = PID.PIDF_Power();
//            chassis.setPower(-power, -power, -power, -power);
//
//            if (Math.abs(PID.getError()) < PID.tolerance){
//                chassis.reset();
//
//                // Make a nested method thingy so that I can just put one for all
//                chassis.setPower(0, 0, 0, 0);
//                driveStep = 3;
//
//            }
//        }
    }

    public void Telemetry(){
        telemetry.addData("CurrentPos", -chassis.frontLeft.getCurrPosInches());
    }
}
