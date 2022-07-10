package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class differentialSwerveHardware {
    public CRServo topServo;
    public CRServo bottomServo;

    public Orientation straight = null;

    // Local OpMode members
    com.qualcomm.robotcore.hardware.HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    // Constructor (Creating Object Type)
    public differentialSwerveHardware() {

    }
    public void init(com.qualcomm.robotcore.hardware.HardwareMap hwMap) {
        hardwareMap = hwMap;

        topServo = hardwareMap.get(CRServo.class,"topServo");
        bottomServo = hardwareMap.get(CRServo.class,"bottomServo");

        topServo.setPower(0);
        bottomServo.setPower(0);
    }
}
