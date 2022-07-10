package org.firstinspires.ftc.teamcode.Hardware.BotMechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Motor;

public class Turret {
    public Motor turretMotor;

    public Gamepad gamepad1;
    public Telemetry telemetry;

    public Turret(String name, HardwareMap hardwareMap){

        turretMotor = new Motor(name, 537.7, 3.77953, hardwareMap);

        turretMotor.reset();

        turretMotor.setBreakMode();

        turretMotor.setDirectionReverse();
    }
}
