package org.firstinspires.ftc.teamcode.otherCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware1 {
    // INSTANTIATE MOTORS AND SERVOS
    public DcMotor frontLeftDrive;
    public DcMotor backLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backRightDrive;
    public DcMotor spinnerDuck;
    public DcMotor spinnerBlock;
    public DcMotor armMotor;
    public Servo servo0;
    public Servo servo1;
    
    // INSTANTIATE SENSORS
    
    
    // CREATING HARDWARE MAP 
    HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap) {

        // DEFINE MOTORS AND SERVOS
        frontLeftDrive = hardwareMap.get(DcMotor.class, "motor1");
        backLeftDrive = hardwareMap.get(DcMotor.class, "motor2");
        frontRightDrive = hardwareMap.get(DcMotor.class, "motor3");
        backRightDrive = hardwareMap.get(DcMotor.class, "motor4");
        spinnerDuck = hardwareMap.get(DcMotor.class, "motor5");
        
        spinnerBlock = hardwareMap.get(DcMotor.class, "motor6");
        armMotor = hardwareMap.get(DcMotor.class, "motor7");
        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        
        // DEFINE SENSORS
        //sensorLeft = hardwareMap.get(DistanceSensor.class, "sensorLeft");
        //sensorCenter = hardwareMap.get(DistanceSensor.class, "sensorCenter");
        //sensorRight = hardwareMap.get(DistanceSensor.class, "sensorRight");

        // SET MOTOR POWER
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        spinnerDuck.setPower(0);
        spinnerBlock.setPower(0);
        armMotor.setPower(0);

        // SET MOTOR MODE 
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinnerDuck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinnerBlock.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // SET MOTOR zeroPowerBehavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinnerDuck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinnerBlock.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // SET SERVO POSITION
        servo0.setPosition(0);
        servo1.setPosition(0);
        
        // CALIBRATE SENSORS
        
    }
 }

