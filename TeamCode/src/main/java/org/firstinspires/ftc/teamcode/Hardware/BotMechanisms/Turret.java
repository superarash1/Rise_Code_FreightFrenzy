package org.firstinspires.ftc.teamcode.Hardware.BotMechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Motor;

public class Turret {
    public Motor turretMotor;

    public Gamepad gamepad1;
    public Telemetry telemetry;

    double turretAngle = 0;
    int circleCondition = 0;

    public Turret(String name, HardwareMap hardwareMap){

        turretMotor = new Motor(name, 537.7, 3.77953, hardwareMap);

        turretMotor.reset();

        turretMotor.setBreakMode();

        turretMotor.setDirectionReverse();
    }

    public double TurretAngle(){
        if (turretAngle > 180){
            circleCondition -= 360;
        }

        if (turretAngle < -180){
            circleCondition += 360;
        }

        turretAngle = turretMotor.getCurrPosDegrees() + circleCondition;

        return turretAngle;
    }
}
