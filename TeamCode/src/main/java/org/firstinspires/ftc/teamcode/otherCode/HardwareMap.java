package org.firstinspires.ftc.teamcode.otherCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HardwareMap {

    // Create Motors
    public DcMotor frontRightMotor;
    public DcMotor frontLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor intake;
    public DcMotor launcher;

    // Create Servo
    public Servo wobbleGoalGrabber;
    public CRServo zipTieSpinner;

    // Create Sensors
    public DistanceSensor backDistance;
    public DistanceSensor frontDistance;
    public BNO055IMU imu;

    public DistanceSensor sensorFront;
    public DistanceSensor sensorRight;
    public DistanceSensor sensorLeft;
    public DistanceSensor sensorBack;

    public Orientation straight = null;

    // local OpMode members.
    com.qualcomm.robotcore.hardware.HardwareMap hardwareMap = null;

    public ElapsedTime runtime = new ElapsedTime();

    // Constructor
    public HardwareMap() {

    }

    public void init(com.qualcomm.robotcore.hardware.HardwareMap hwMap){
        hardwareMap = hwMap;

        // Connect Drivetrain Motors
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");

        // Connect Other Motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotor.class, "flywheel");

        // Connect Servos
        wobbleGoalGrabber = hardwareMap.get(Servo.class, "wobble");
        zipTieSpinner = hardwareMap.get(CRServo.class, "fire");

        // Connect Sensors
        sensorFront = hardwareMap.get(DistanceSensor.class, "sensorFront");
        //sensorRight = hardwareMap.get(DistanceSensor.class, "sensorRight");
        sensorLeft = hardwareMap.get(DistanceSensor.class, "sensorLeft");
        //sensorBack = hardwareMap.get(DistanceSensor.class, "sensorBack");*/
        imu = hardwareMap.get(BNO055IMU.class, "imu");



        // Set all motors to zero power
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        launcher.setPower(0);
        intake.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //idk what this line does

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelUnit = BNO055IMU.AccelUnit.MILLI_EARTH_GRAVITY; //might use that actually

        imu.initialize(parameters);

        straight = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Changed Axes Order

        // Motor Direction
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Start encoders @ d = 0
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to run with encoders.
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
