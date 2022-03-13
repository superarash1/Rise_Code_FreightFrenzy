package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.outoftheboxrobotics.neutrinoi2c.Rev2mDistanceSensor.AsyncRev2MSensor;

public class Hardware {

    // Define Motor Variables
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor backLeftMotor;

    public DcMotor intake;
    public DcMotorEx arm1;
    public DcMotorEx arm2;
    public DcMotorEx cSpinRight;


    public CRServoImplEx cageSpin1;

    public Servo gate;

    public CRServo capper;

    // Define Sensor Variables
    public BNO055IMU imu;
//    public Rev2mDistanceSensor sensor;


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

        // Find Motors in config
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");


        intake = hardwareMap.get(DcMotor.class, "intake");
        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
        arm2 = hardwareMap.get(DcMotorEx.class, "arm2");
        cSpinRight = hardwareMap.get(DcMotorEx.class, "carouselSpinner");

        cageSpin1 = hardwareMap.get(CRServoImplEx.class, "cageSpin1");
        capper = hardwareMap.get(CRServo.class, "capper");
//        cageSpin2 = hardwareMap.get(CRServo.class, "cageSpin2");

//        cSpinRight = hardwareMap.get(CRServo.class, "carouselSpinnerLeft");

        gate = hardwareMap.get(Servo.class, "gate");


//
//        backDistance = hardwareMap.get(DistanceSensor.class, "backDistance");

        //Find Sensors in phone config
//        sensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);

        intake.setPower(0);
        arm1.setPower(0);
        arm2.setPower(0);
        cSpinRight.setPower(0);

        capper.setPower(0);

        cageSpin1.setPower(0);
//        cageSpin2.setPower(0);
//
//        cSpinRight.setPower(0);

        // IMU Setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelUnit = BNO055IMU.AccelUnit.MILLI_EARTH_GRAVITY;
            /*
            We never use the accelerometer functions of the imu, so we set the
            accel unit to milli-e/arth-gravities as a joke
            */

        imu.initialize(parameters);

        straight = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Set Motor Direction
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);


        intake.setDirection(DcMotor.Direction.FORWARD);
        arm1.setDirection(DcMotor.Direction.REVERSE);
        arm2.setDirection(DcMotor.Direction.FORWARD);

        // Set the Motor's Encoder Setting
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cSpinRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void resetEncoders(){
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setBreakMode(){
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setFloatMode(){
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // Constants to find the amount of encoder ticks per CM
    double COUNTS_PER_MOTOR_REV = 537.7;
    double DRIVE_GEAR_REDUCTION = 1;
    double WHEEL_DIAMETER_INCH = 3.77953;

    // Finds the amount of encoder ticks per CM
    double TICKS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH * 3.1415);

    public double getCurrPosInches(){
        return (frontRightMotor.getCurrentPosition() / TICKS_PER_INCH);
    }


}


