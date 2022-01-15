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
    public CRServo duck1;
    public CRServo duck2;

    com.qualcomm.robotcore.hardware.HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    // Constructor (Creating Object Type)
    public mattHServo() {

    }

    // Initialize standard Hardware interfaces
    public void init(com.qualcomm.robotcore.hardware.HardwareMap hwMap) {
        hardwareMap = hwMap;

        duck1 = hardwareMap.get(CRServo.class, "duck1");
        duck2 = hardwareMap.get(CRServo.class, "duck2");
    }
}
