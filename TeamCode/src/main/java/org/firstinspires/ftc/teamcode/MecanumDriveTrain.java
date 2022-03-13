package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDriveTrain {
    //Declare the variables for the mecanum drive train class
    /* Although the encoders aren't DcMotors, they can be initialized as one
    since they are connected to the drive train motor encoder ports on the rev hub.
    For Example, if the Left Encoder is attached to the port associated with the tl
    Motor , then the hardware name for the Left Encoder and the tl Motor would be the same
    */

    private Motor frontLeft;
    private Motor frontRight;
    private Motor backRight;
    private Motor backLeft;

    double drive;
    double turn;
    double strafe;
    double fLeft;
    double fRight;
    double bLeft;
    double bRight;
    double max;

    boolean switchMode1 = true;
    boolean switchMode2 = false;

    boolean lastToggleA = true;
    boolean toggle1 = true;
    boolean toggle2 = true;

    public BNO055IMU imu;

    private Telemetry telemetry;
    private Gamepad gamepad1;
    private HardwareMap hardwareMap;


    public Orientation straight = null;
    public double globalAngle;

    PIDF_Controller PID = new PIDF_Controller();

    public MecanumDriveTrain(String flName, String frName, String brName, String blName, HardwareMap hardwareMap , Telemetry telemetry, Gamepad gamepad1) {
        this.frontLeft = new Motor(flName, 537.7, 3.77953, hardwareMap);
        this.frontRight = new Motor(frName, 537.7, 3.77953, hardwareMap);
        this.backRight = new Motor(brName, hardwareMap);
        this.backLeft = new Motor(blName, hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

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

        this.frontLeft.reset();
        this.frontRight.reset();
        this.backRight.reset();
        this.backLeft.reset();

        // This part is depends on your preference for zero power behavior. For the implementation on my robot, I will set it to break mode.
        this.frontLeft.setBreakMode();
        this.frontRight.setBreakMode();
        this.backRight.setBreakMode();
        this.backLeft.setBreakMode();

        this.frontLeft.setDirectionReverse();
        this.frontRight.setDirectionForward();
        this.backRight.setDirectionForward();
        this.backLeft.setDirectionReverse();

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
    }

    public double calculateTotalAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = angles.firstAngle - straight.firstAngle;

        return -globalAngle;
    }

    public void outputEncoderReadings(){
        telemetry.addData("Left Encoder Position Inches : ", frontLeft.getCurrPosInches());
        telemetry.addData("Right Encoder Position Inches : ", frontRight.getCurrPosInches());
        telemetry.addData("IMU Angle Degrees : ", calculateTotalAngle());
        telemetry.update();
    }

    public void teleOpDrive(){

        drive = Math.pow(gamepad1.left_stick_y, 3); //Between -1 and 1
        turn = Math.pow(gamepad1.right_stick_x, 3);
        strafe = Math.pow(gamepad1.left_stick_x, 3);

        if ((gamepad1.a != lastToggleA) && gamepad1.a && toggle1){
            switchMode1 = false;
            switchMode2 = true;

            toggle1 = false;
            toggle2 = true;
        } else if ((gamepad1.a != lastToggleA) && gamepad1.a && toggle2){
            switchMode2 = false;
            switchMode1 = true;

            toggle2 = false;
            toggle1 = true;
        }
        lastToggleA = gamepad1.a;

        // Mecanum Drive Calculations
        if (switchMode1){
            fLeft = 0.875 * drive - 1 * strafe + 0.8 * turn;
            fRight = 0.875 * drive + 1 * strafe - 0.8 * turn;
            bRight = 0.875 * drive - 1 * strafe - 0.8 * turn;
            bLeft = 0.875 * drive + 1 * strafe + 0.8 * turn;
        } else if (switchMode2){
            fLeft = -0.875 * drive + 1 * strafe + 0.8 * turn;
            fRight = -0.875 * drive - 1 * strafe - 0.8 * turn;
            bRight = -0.875 * drive + 1 * strafe - 0.8 * turn;
            bLeft = -0.875 * drive - 1 * strafe + 0.8 * turn;
        }

        // This ensures that the power values the motors are set to are in the range (-1, 1)
        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }

        this.frontLeft.setPower(fLeft);
        this.frontRight.setPower(fRight);
        this.backRight.setPower(bRight);
        this.backLeft.setPower(bLeft);
    }

    public void PID_Drive(double target, double tolerance){
        double power;
        while (PID.getError() > tolerance){
            power = PID.PIDF_Drive(frontLeft.getCurrPosInches(), target, 1, 0, 0, 0);
            setPower(power, power, power, power);
        }
        reset();
    }

    public void PID_Turn(double target, double tolerance){
        double power;
        while (PID.getError() > tolerance){
            power = PID.PIDF_Drive(calculateTotalAngle(), target, 1, 0, 0, 0);
            setPower(power, -power, -power, power);
        }
        reset();
    }

    public void PID_Strafe(double target, double tolerance){
        double power;
        while (PID.getError() > tolerance){
            power = PID.PIDF_Drive(frontLeft.getCurrPosInches(), target, 1, 0, 0, 0);
            setPower(-power, power, -power, power);
        }
        reset();
    }

    public void setPower(double fLeft, double fRight, double bRight, double bLeft){
        this.frontLeft.setPower(fLeft);
        this.frontRight.setPower(fRight);
        this.backRight.setPower(bRight);
        this.backLeft.setPower(bLeft);
    }

    public void reset(){
        this.frontLeft.reset();
        this.frontRight.reset();
        this.backRight.reset();
        this.backLeft.reset();
    }
}
