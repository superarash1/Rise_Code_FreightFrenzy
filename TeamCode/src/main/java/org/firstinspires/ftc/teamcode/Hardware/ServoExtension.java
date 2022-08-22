package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.hardware.hitechnic.HiTechnicNxtIrSeekerSensor.MAX_ANGLE;
import static com.qualcomm.hardware.hitechnic.HiTechnicNxtIrSeekerSensor.MIN_ANGLE;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ServoExtension {
    public ServoEx servo;

    public ServoExtension(String servoName, HardwareMap hardwareMap){
        servo = new SimpleServo(hardwareMap, servoName, MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
    }
}
