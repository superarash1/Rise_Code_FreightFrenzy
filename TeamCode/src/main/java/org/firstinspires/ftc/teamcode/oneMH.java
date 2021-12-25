package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class oneMH {

    public DcMotor cowMotor;

    com.qualcomm.robotcore.hardware.HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    public oneMH() {

    }
    public void init(com.qualcomm.robotcore.hardware.HardwareMap hwMap) {
        hardwareMap = hwMap;

        cowMotor = hardwareMap.get(DcMotor.class, "cowMotor");
        cowMotor.setDirection(DcMotor.Direction.FORWARD);
        cowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}

