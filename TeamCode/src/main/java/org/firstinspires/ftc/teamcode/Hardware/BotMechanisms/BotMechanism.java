package org.firstinspires.ftc.teamcode.Hardware.BotMechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Motor;
import org.firstinspires.ftc.teamcode.PIDF_Controller;

public class BotMechanism {
    private Motor intake;
    private Motor armLeft;
    private Motor armRight;

//    private CRServo cageSpin;
    private ServoImplEx cageSpin;
    private Servo gate;
    private CRServo carouselSpinner;

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

    public enum armState {
        HOME,
        TOP_LEVEL,
        MIDDLE_LEVEL,
        BOTTOM_LEVEL
    }

    public enum cageState {
        DROP,
        FLAT,
        MANUAL
    }

    public enum gateState {
        OPEN,
        CLOSED
    }

    PIDF_Controller PIDF;

    public armState ArmState;
    cageState CageState;
    gateState GateState;

    // TODO: Make everything with run using encoder

    public BotMechanism(String intake, String Arm1, String Arm2, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1){
        this.intake = new Motor(intake, 537.7, 0, hardwareMap);
        this.armLeft = new Motor(Arm1, 537.7, 3.77953, hardwareMap);
        this.armRight = new Motor(Arm2, 537.7, 3.77953, hardwareMap);

        cageSpin = hardwareMap.get(ServoImplEx.class, "cageSpin");
        gate = hardwareMap.get(Servo.class, "gate");
        carouselSpinner = hardwareMap.get(CRServo.class, "cSpin");

        PIDF = new PIDF_Controller(0.6, telemetry);

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

        ArmState = armState.HOME;
        CageState = cageState.FLAT;
        GateState = gateState.CLOSED;
    }

    double firstError = 5;

    public void Intake(){
        this.intake.setPower(Math.pow(gamepad1.right_trigger - gamepad1.left_trigger, 3));
    }

    public void Arm(){

        this.armLeft.setPower(PIDF.PIDF_Arm(armLeft.getCurrPosDegrees(), targetPos, 0.6,0, 0,0,0.03, firstError));
        this.armRight.setPower(PIDF.PIDF_Arm(armLeft.getCurrPosDegrees(), targetPos, 0.6,0, 0,0,0.03, firstError));

        switch (ArmState){
            case HOME:
                targetPos = -33;
                firstError = targetPos - armLeft.getCurrPosDegrees();

                CageState = cageState.FLAT;
                if ((gamepad1.x != lastToggleX) && gamepad1.x && armToggle1){
                    ArmState = armState.TOP_LEVEL;

                    armToggle1 = false;
                    armToggle2 = true;
                }
                break;
            case TOP_LEVEL:
                targetPos = 135;

                firstError = 178.8;
                CageState = cageState.DROP;

                if ((gamepad1.x != lastToggleX) && gamepad1.x && armToggle2){
                    ArmState = armState.HOME;

                    armToggle1 = true;
                    armToggle2 = false;
                }

                break;
            case MIDDLE_LEVEL:
                targetPos = 175;
                firstError = targetPos - armLeft.getCurrPosDegrees();
                CageState = cageState.FLAT;

                break;
            case BOTTOM_LEVEL:
                targetPos = 205;
                firstError = targetPos - armLeft.getCurrPosDegrees();
                CageState = cageState.FLAT;

                break;
        }


            //This is the working block of code \/\/ (I think)
        if ((gamepad1.x != lastToggleX) && gamepad1.x && armToggle1) {
            ArmState = armState.TOP_LEVEL;
            armToggle1 = false;
            armToggle2 = true;

        } else if ((gamepad1.x != lastToggleX) && gamepad1.x && armToggle2) {
            ArmState = armState.HOME;
            boxTarget = 0.7;

            armToggle2 = false;
            armToggle1 = true;
        }

        lastToggleX = gamepad1.x;

//        if ((gamepad1.x != lastToggleX) && gamepad1.x && armToggle1) {
//            targetPos = 135;
//            firstError = 178.8;
//
//            boxTarget = 0;
//
//            armToggle1 = false;
//            armToggle2 = true;
//
//        } else if ((gamepad1.x != lastToggleX) && gamepad1.x && armToggle2) {
//            targetPos = -33;
//            firstError = targetPos - armLeft.getCurrPosDegrees();
//
//            boxTarget = 0.7;
//
//            armToggle2 = false;
//            armToggle1 = true;
//        }
//
//        lastToggleX = gamepad1.x;
//
//        if (gamepad1.dpad_up && armLeft.getCurrPosDegrees() > 100 && armLeft.getCurrPosDegrees() < 160 && armCoolDown > 100){
//            targetPos = 175;
//            firstError = targetPos - armLeft.getCurrPosDegrees();
//            boxTarget = 0;
//
//            armCoolDown = 0;
//        }
//
//        if (gamepad1.dpad_up && armLeft.getCurrPosDegrees() > 160 && armCoolDown > 100){
//            targetPos = 205;
//            firstError = targetPos - armLeft.getCurrPosDegrees();
//            boxTarget = 0;
//
//            armCoolDown = 0;
//        }
    }

    public void carouselSetPower(double power){
        carouselSpinner.setPower(power);
    }

    public void cageRotation(){
        cageSpin.setPosition(boxTarget);

        switch (CageState){
            case FLAT:
                boxTarget = 0;

                break;
            case DROP:
                boxTarget = 0.7;

                break;
            case MANUAL:
//                if (gamepad1.right_bumper) {
//                    cageSpin.setPosition(boxTarget - 0.01);
//                } else if (gamepad1.left_bumper) {
//                    cageSpin.setPosition(boxTarget + 0.01);
//                }

                if (gamepad1.right_bumper) {
                    boxTarget = boxTarget - 0.01;
                } else if (gamepad1.left_bumper) {
                    boxTarget = boxTarget - 0.01;;
                }

                break;
        }
    }
    public void toggleGate(){

        /* TODO: Might need to switch open and closed positions cuz I don't remember which one is which
            (including initial state in constructor)
         */
        switch (GateState) {
            case OPEN:
                gate.setPosition(0.6);

                break;
            case CLOSED:
                gate.setPosition(0);

                break;
        }

        if ((gamepad1.y != lastToggleY) && gamepad1.y && gateToggle1) {
            GateState = gateState.OPEN;
            gateToggle1 = false;
            gateToggle2 = true;

        } else if ((gamepad1.y != lastToggleY) && gamepad1.y && gateToggle2) {
            GateState = gateState.CLOSED;
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
//        telemetry.addData("State", state);
        telemetry.addData("Angle", this.armLeft.getCurrPosDegrees());
        telemetry.addData("Target Position", targetPos);

        telemetry.addData("Error", PIDF.getError());

        telemetry.addData("First Error", firstError);

        telemetry.update();
    }
}
