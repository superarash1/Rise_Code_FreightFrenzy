package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BotMechanisms {
    private Motor intake;
    private Motor armLeft;
    private Motor armRight;

//    private CRServo cageSpin;
    private ServoImplEx cageSpin;
    private Servo gate;
    private CRServo carouselSpinner;

    PIDF_Controller PIDF;

    private Telemetry telemetry;
    private Gamepad gamepad1;
    private HardwareMap hardwareMap;

    boolean armToggle1 = true;
    boolean armToggle2 = false;

    boolean gateToggle1 = true;
    boolean gateToggle2 = false;

    boolean lastToggleY = true;
    boolean lastToggleX = true;

    double armCoolDown = 200;
    double gateCoolDown = 200;
    double targetPos = -33;
    public double boxTarget = 0.7;

    public BotMechanisms(String intake, String Arm1, String Arm2, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1){
        this.intake = new Motor(intake, 537.7, 0, hardwareMap);
        this.armLeft = new Motor(Arm1, 537.7, 3.77953, hardwareMap);
        this.armRight = new Motor(Arm2, 537.7, 3.77953, hardwareMap);

        PIDF = new PIDF_Controller(1,0.15, telemetry);
        cageSpin = hardwareMap.get(ServoImplEx.class, "cageSpin");
        gate = hardwareMap.get(Servo.class, "gate");
        carouselSpinner = hardwareMap.get(CRServo.class, "cSpin");

        this.intake.reset();
        this.armLeft.reset();
        this.armRight.reset();

        this.armLeft.setBreakMode();
        this.armRight.setBreakMode();

        armLeft.setDirectionReverse();
        armRight.setDirectionReverse();

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;


    }



    double firstError = 5;

    public void Intake(){
        this.intake.setPower(Math.pow(gamepad1.right_trigger - gamepad1.left_trigger, 3));

//        if (gamepad1.b){
//            this.intake.runToPosition(1510);
//            this.intake.reset();
//        }
    }

    public void Arm(){

        this.armLeft.setPower(PIDF.PIDF_Arm(armLeft.getCurrPosDegrees(), targetPos, 1,0.15, 0,0,0.02, firstError));
        this.armRight.setPower(PIDF.PIDF_Arm(armLeft.getCurrPosDegrees(), targetPos, 1,0.15, 0,0,0.02, firstError));

        if ((gamepad1.x != lastToggleX) && gamepad1.x && armToggle1) {
            targetPos = 135;
            firstError = 178.8;

            boxTarget = 0;

            armToggle1 = false;
            armToggle2 = true;

        } else if ((gamepad1.x != lastToggleX) && gamepad1.x && armToggle2) {
            targetPos = -33;
            firstError = targetPos - armLeft.getCurrPosDegrees();

            boxTarget = 0.7;

            armToggle2 = false;
            armToggle1 = true;
        }

        lastToggleX = gamepad1.x;

//        if (gamepad1.x && armToggle1 && armCoolDown > 100){
//            targetPos = 144;
//            firstError = 178.8;
//
//            armToggle1 = false;
//            armToggle2 = true;
//
//            armCoolDown = 0;
//        }
//
//        if (gamepad1.x && armToggle2 && armCoolDown > 100){
//            targetPos = -36;
//            firstError = targetPos - armLeft.getCurrPosDegrees();
//
//            armToggle2 = false;
//            armToggle1 = true;
//
//            armCoolDown = 0;
//        }

        if (gamepad1.dpad_up && armLeft.getCurrPosDegrees() > 100 && armLeft.getCurrPosDegrees() < 160 && armCoolDown > 100){
            targetPos = 175;
            firstError = targetPos - armLeft.getCurrPosDegrees();
            boxTarget = 0;

            armCoolDown = 0;
        }

        if (gamepad1.dpad_up && armLeft.getCurrPosDegrees() > 160 && armCoolDown > 100){
            targetPos = 205;
            firstError = targetPos - armLeft.getCurrPosDegrees();
            boxTarget = 0;

            armCoolDown = 0;
        }

//        if (gamepad1.left_bumper && armLeft.getCurrPosDegrees() > 210 && armCoolDown > 100){
//            targetPos = 175;
//            firstError = targetPos - armLeft.getCurrPosDegrees();
//
//            armCoolDown = 0;
//        }
//
//        if (gamepad1.left_bumper && armLeft.getCurrPosDegrees() > 180 && armLeft.getCurrPosDegrees() < 210 && armCoolDown > 100){
//            targetPos = 144;
//            firstError = targetPos - armLeft.getCurrPosDegrees();
//
//            armCoolDown = 0;
//        }

    }
//
//    public void cageRotation(){
//        if (gamepad1.dpad_right) {
//            cageSpin.setPower(-0.3);
//        } else if (gamepad1.dpad_left) {
//            cageSpin.setPower(0.3);
//        } else {
//            cageSpin.setPower(0);
//        }
//    }

    public void carouselSetPower(double power){
        carouselSpinner.setPower(power);
    }

    public void cageRotation(){
        cageSpin.setPosition(boxTarget);

        if (gamepad1.right_bumper) {
            cageSpin.setPosition(boxTarget - 0.01);
        } else if (gamepad1.left_bumper) {
            cageSpin.setPosition(boxTarget + 0.01);
        }


    }
    public void toggleGate(){
        if ((gamepad1.y != lastToggleY) && gamepad1.y && gateToggle1) {
            gate.setPosition(0.6);
            gateToggle1 = false;
            gateToggle2 = true;

        } else if ((gamepad1.y != lastToggleY) && gamepad1.y && gateToggle2) {
            gate.setPosition(0);
            gateToggle1 = true;
            gateToggle2 = false;
        }
        lastToggleY = gamepad1.y;
    }

    public void setGatePosition(double target){
        gate.setPosition(target);
    }
    public void Telemetry(){
        telemetry.addData("Cage Spin Position:", cageSpin.getPosition());
//        telemetry.addData("Gate Position:", gate.getPosition());
//        telemetry.addData("intake position", this.intake.getPositionTicks());
        telemetry.addData("Angle:", this.armLeft.getCurrPosDegrees());
        telemetry.addData("Target Position", targetPos);

//        telemetry.addData("Delta Time", PIDF.getDeltaTime());

        telemetry.update();
    }
}
