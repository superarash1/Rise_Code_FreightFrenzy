package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor {
    DcMotorEx motor;

    //Declare all the constants in the Motor class
    private double CPR;
    private double WHEEL_DIAMETER;
    private double MAX_RPM;
    private double TICKS_PER_INCH;
    private double ARM_COUNTS_PER_DEGREE;
    private double NANOSECONDS_PER_MIN = 6e+10;

       /* Constructor for drive train motors
       Parameter name : Pass in name of the motor on the RC phone config
       Parameter hwmap : Pass in the hardwareMap from OpMode to initialize the motor */

    /* Constructor for drive train motors
       Parameter name : Pass in name of the motor on the RC phone config
       Parameter hwmap : Pass in the hardwareMap from OpMode to initialize the motor */

    public Motor(String name, HardwareMap hwmap){
        motor = hwmap.get(DcMotorEx.class, name);
    }

    /* Constructor for dead wheel encoders
       Parameter name : Name of the motor connected to the respective encoder port
       Parameter cpr : Encoder ticks per one revolution
       Parameter wheelDiameter : Diameter of the dead wheel
       Parameter hwmap : Pass in the hardwareMap from OpMode to initialize the motor */

    public Motor(String name , double cpr , double wheelDiameter, HardwareMap hwmap){
        motor = hwmap.get(DcMotorEx.class, name);
        this.CPR = cpr;
        this.WHEEL_DIAMETER = wheelDiameter;
        this.TICKS_PER_INCH = cpr / (wheelDiameter * Math.PI);
        this.ARM_COUNTS_PER_DEGREE = (cpr / 360);
    }

    public void reset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runToPosition(int target){
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(0.1);

        while (motor.isBusy()){

        }
        reset();
    }

    public void setBreakMode(){
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setFloatMode(){
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public double getCurrPosInches(){
        return motor.getCurrentPosition() / TICKS_PER_INCH;
    }
    public double getCurrPosDegrees(){
        return -0.23809*(motor.getCurrentPosition()/ARM_COUNTS_PER_DEGREE) - 38;
    }
    public double getPositionTicks(){
        return motor.getCurrentPosition();
    }

    public void setDirectionForward(){
        motor.setDirection(DcMotor.Direction.FORWARD);
    }
    public void setDirectionReverse(){
        motor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setPower(double power){
        motor.setPower(power);
    }
}
