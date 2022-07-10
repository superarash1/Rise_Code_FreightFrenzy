package org.firstinspires.ftc.teamcode.Hardware.BotMechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Motor;

public class Intake {

    public Motor intakeMotor;

    public Intake(String name, HardwareMap hardwareMap){
        intakeMotor = new Motor(name, 537.7, 0, hardwareMap); // "intake"

        intakeMotor.reset();
    }

    public double getPosTicks(){
        return intakeMotor.getPositionTicks();
    }
}
