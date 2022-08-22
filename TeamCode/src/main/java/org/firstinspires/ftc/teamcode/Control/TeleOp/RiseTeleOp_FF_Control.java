package org.firstinspires.ftc.teamcode.Control.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.MotionControl.PIDF_Controller;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.Arm;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.Cage;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.Gate;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.Intake;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.MecanumDriveTrain;

public class RiseTeleOp_FF_Control {

    MecanumDriveTrain driveTrain;
    Arm armLeft;
    Arm armRight;
    Gate gate;
    Intake intake;
    Cage cage;

    Gamepad gamepad1;
    Telemetry telemetry;

    double drive;
    double turn;
    double strafe;
    double fLeft;
    double fRight;
    double bLeft;
    double bRight;
    double max;

    double antiGravity;
    double kGravity = 0.03;

    double targetPos;
    double maxError;

    double armPower;

    boolean switchMode1 = true;
    boolean switchMode2 = false;

    boolean armToggle1 = true;
    boolean armToggle2 = false;
    boolean armToggle3 = true;
    boolean armToggle4 = false;
    boolean armToggle5 = true;
    boolean armToggle6 = false;

    boolean toggle1 = true;
    boolean toggle2 = false;
    boolean gateToggle1 = true;
    boolean gateToggle2 = false;

    boolean lastToggleA = true;
    boolean lastToggleX = true;
    boolean lastToggleDown = true;
    boolean lastToggleUp = true;
    boolean lastToggleY = true;

    public enum cageState {
        DROP,
        FLAT,
        MANUAL
    }

    public enum armState {
        HOME,
        TOP_LEVEL,
        MIDDLE_LEVEL,
        BOTTOM_LEVEL
    }

    public armState ArmState;
    public cageState CageState;

    PIDF_Controller ArmPID;

    public double boxTarget = 0.7;

    public RiseTeleOp_FF_Control(String flName, String frName, String brName, String blName, String Arm1, String Arm2, String CageName, String GateName, String IntakeName, HardwareMap hardwareMap , Telemetry telemetry, Gamepad gamepad1){
        driveTrain = new MecanumDriveTrain(flName, frName, brName, blName, hardwareMap);
        armLeft = new Arm(Arm1, -38, hardwareMap, telemetry);
        armRight = new Arm(Arm2, -38, hardwareMap, telemetry);
        gate = new Gate(GateName, hardwareMap, 0.6);
        intake = new Intake(IntakeName, hardwareMap);
        cage = new Cage(CageName, hardwareMap);

        ArmPID = new PIDF_Controller(1, telemetry);

        driveTrain.setBreakMode();
        driveTrain.reset();

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;

        ArmState = armState.HOME;
        CageState = cageState.FLAT;
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
            fLeft = 0.875 * drive - 1 * strafe - 0.8 * turn;
            fRight = 0.875 * drive + 1 * strafe + 0.8 * turn;
            bRight = 0.875 * drive - 1 * strafe + 0.8 * turn;
            bLeft = 0.875 * drive + 1 * strafe - 0.8 * turn;
        } else if (switchMode2){
            fLeft = -0.875 * drive + 1 * strafe - 0.8 * turn;
            fRight = -0.875 * drive - 1 * strafe + 0.8 * turn;
            bRight = -0.875 * drive + 1 * strafe + 0.8 * turn;
            bLeft = -0.875 * drive - 1 * strafe - 0.8 * turn;
        }

        // This ensures that the power values the motors are set to are in the range (-1, 1)
        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }

        driveTrain.setPower(fLeft, fRight, bRight, bLeft);
    }

    public void Arm(){
        antiGravity = kGravity*(Math.cos(Math.toRadians(armLeft.armAngle())));

        armPower = ArmPID.PIDF_Power(armLeft.armAngle(), targetPos, 360) + antiGravity;

        armLeft.armMotor.setPower(armPower);
        armRight.armMotor.setPower(armPower);

        switch (ArmState){
            case HOME:
                targetPos = -33;

                if ((gamepad1.x != lastToggleX) && gamepad1.x && armToggle1){
                    armToggle1 = false;
                    armToggle2 = true;

                    ArmState = armState.TOP_LEVEL;
                }

                lastToggleX = gamepad1.x;

                CageState = cageState.FLAT;
                break;
            case TOP_LEVEL:
                targetPos = 135;

                if ((gamepad1.x != lastToggleX) && gamepad1.x && armToggle2){
                    armToggle2 = false;
                    armToggle1 = true;

                    ArmState = armState.HOME;
                }

                if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down && armToggle3){
                    armToggle3 = false;
                    armToggle4 = true;

                    ArmState = armState.MIDDLE_LEVEL;
                }

                lastToggleX = gamepad1.x;
                lastToggleDown = gamepad1.dpad_down;

                CageState = cageState.DROP;

                break;
            case MIDDLE_LEVEL:
                targetPos = 175;
                CageState = cageState.FLAT;

                if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down && armToggle5){
                    armToggle5 = false;
                    armToggle6 = true;

                    ArmState = armState.BOTTOM_LEVEL;
                }

                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up && armToggle4){
                    armToggle4 = false;
                    armToggle3 = true;

                    ArmState = armState.TOP_LEVEL;
                }

                lastToggleDown = gamepad1.dpad_down;
                lastToggleUp = gamepad1.dpad_up;

                break;
            case BOTTOM_LEVEL:
                targetPos = 205;
                CageState = cageState.FLAT;

                if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up && armToggle6){
                    armToggle6 = false;
                    armToggle5 = true;

                    ArmState = armState.TOP_LEVEL;
                }
                lastToggleUp = gamepad1.dpad_up;

                break;
        }
    }

    public void cageRotation(){
        cage.cageSpin.setPosition(boxTarget);

        switch (CageState){
            case FLAT:
                boxTarget = 0;

                break;
            case DROP:
                boxTarget = 0.7;

                break;
            case MANUAL:
                if (gamepad1.right_bumper) {
                    boxTarget -= -0.01;
                } else if (gamepad1.left_bumper) {
                    boxTarget += 0.01;
                }
                break;
        }
    }

    public void teleOpGate(){
        switch (gate.GateState){
            case OPEN:
                if ((gamepad1.y != lastToggleY) && gamepad1.y && gateToggle1){
                    gateToggle1 = false;
                    gateToggle2 = true;

                    gate.GateState = Gate.gateState.CLOSED;
                }

                lastToggleY = gamepad1.y;
                break;
            case CLOSED:
                if ((gamepad1.y != lastToggleY) && gamepad1.y && gateToggle2){
                    gateToggle2 = false;
                    gateToggle1 = true;

                    gate.GateState = Gate.gateState.OPEN;
                }

                lastToggleY = gamepad1.y;
                break;
        }
        gate.toggleGate();
    }

    public void teleOpIntake(){
        intake.intakeMotor.setPower(Math.pow(gamepad1.right_trigger - gamepad1.left_trigger, 3));
    }
}
