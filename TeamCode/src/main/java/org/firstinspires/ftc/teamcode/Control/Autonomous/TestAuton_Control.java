package org.firstinspires.ftc.teamcode.Control.Autonomous;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.Turret;
import org.firstinspires.ftc.teamcode.PIDF_Controller;

public class TestAuton_Control {

    public MecanumDriveTrain chassis;
    public Turret turret;

    public Telemetry telemetry;
    public Gamepad gamepad1;
    public HardwareMap hardwareMap;

    public int driveStep = 1;

    public double power;

    public PIDF_Controller PID;

    public TestAuton_Control(String flName, String frName, String brName, String blName, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1){

        this.PID = new PIDF_Controller(telemetry);
        this.chassis = new MecanumDriveTrain(flName, frName, brName, blName, hardwareMap);
        this.turret = new Turret("spinny", hardwareMap);

        PID.tolerance = 0.05;//0.05

        this.chassis.reset();

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
    }

    public void Turret(){
        if (driveStep == 1){
            turret.turretMotor.setPower(1);
        }else if (driveStep == 2){
            turret.turretMotor.setPower(-1);
        }else {
            turret.turretMotor.setPower(0);
        }
    }

    public void Drive(){
        if (driveStep == 1){

            power = PID.PIDF(chassis.frontLeft.getCurrPosInches(), 15, 0.6, 0.0032, 0, 0.06); // 0.675, 0.0032, 0, 0.01

            chassis.setPower(-power, -power, -power, -power);

            if (PID.getError() < PID.tolerance){
                chassis.reset();

                // Make a nested method thingy so that I can just put one for all
                chassis.setPower(0, 0, 0, 0);
                driveStep = 2;

            }
        }

        if (driveStep == 2){

            power = PID.PIDF(chassis.frontLeft.getCurrPosInches(), -15, 0.6, 0.0032, 0, 0.06);
            chassis.setPower(-power, -power, -power, -power);

            if (Math.abs(PID.getError()) < PID.tolerance){
                chassis.reset();

                // Make a nested method thingy so that I can just put one for all
                chassis.setPower(0, 0, 0, 0);
                driveStep = 3;

            }
        }
    }

    public void Telemetry(){
        telemetry.addData("Auton Step", driveStep);

        telemetry.addData("Power", power);
    }
}
