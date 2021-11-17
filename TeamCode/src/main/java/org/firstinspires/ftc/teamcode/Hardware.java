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

public class Hardware {

    // Define Motor Variables
    public DcMotor frontRightMotor;
    public DcMotor frontLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor middleLeftMotor;
    public DcMotor middleRightMotor;
    public DcMotor turretSpinner;
    public DcMotor carouselSpinner;

    // Define Servo Variables
    public Servo turret;

    // Define Sensor Variables
    public BNO055IMU imu;
    public Orientation straight = null;

    // Local OpMode members
    com.qualcomm.robotcore.hardware.HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    // Constructor (Creating Object Type)
    public Hardware() {

    }

    // Initialize standard Hardware interfaces
    public void init(com.qualcomm.robotcore.hardware.HardwareMap hwMap){
        hardwareMap = hwMap;

        // Find Motors in phone config
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        middleLeftMotor = hardwareMap.get(DcMotor.class, "middleLeft");
        middleRightMotor = hardwareMap.get(DcMotor.class, "middleRight");
        carouselSpinner = hardwareMap.get(DcMotor.class, "carouselSpinner");
        turretSpinner = hardwareMap.get(DcMotor.class, "turretSpinner");


        // Find Servos in phone config
        turret = hardwareMap.get(Servo.class,"turret");


        //Find Sensors in phone config
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        // Set all motors to zero power
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        middleLeftMotor.setPower(0);
        middleRightMotor.setPower(0);
        carouselSpinner.setPower(0);
        turretSpinner.setPower(0);

        // IMU Setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelUnit = BNO055IMU.AccelUnit.MILLI_EARTH_GRAVITY;
            /*
            We never use the accelerometer functions of the imu, so we set the
            accel unit to milli-earth-gravities as a joke
            */

        imu.initialize(parameters);

        straight = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        // Set Motor Direction
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        middleLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        middleRightMotor.setDirection(DcMotor.Direction.FORWARD);

        carouselSpinner.setDirection(DcMotor.Direction.FORWARD);
        turretSpinner.setDirection(DcMotor.Direction.FORWARD);

        // Set the Motor's Encoder Setting
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        carouselSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
}


