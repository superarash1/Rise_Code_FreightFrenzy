package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class mattHServo {

    com.qualcomm.robotcore.hardware.HardwareMap hardwareMap;
    public ElapsedTime runtime = new ElapsedTime();


    public CRServo carSpinL;
    //    public CRServo duck2;
    //    open and close
    public Servo boxO;
//    box turning
    public CRServo boxT1;
    public CRServo boxT2;


    // Constructor (Creating Object Type)
    public mattHServo() {

    }

    // Initialize standard Hardware interfaces
    public void init(com.qualcomm.robotcore.hardware.HardwareMap hwMap) {
        hardwareMap = hwMap;

        carSpinL = hardwareMap.get(CRServo.class, "carSpinL");
//        duck2 = hardwareMap.get(CRServo.class, "duck2");
        boxO = hardwareMap.get(Servo.class, "boxO");
        boxT1 = hardwareMap.get(CRServo.class, "boxT1");
        boxT2 = hardwareMap.get(CRServo.class, "boxT2");


    }
}