package org.firstinspires.ftc.teamcode.Control.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.Arm;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.Gate;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.Intake;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.MecanumDriveTrain;

public class RiseTeleOp_FF_Control {

    MecanumDriveTrain driveTrain;
    Arm arm;
    Gate gate;
    Intake intake;

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

    boolean switchMode1 = true;
    boolean switchMode2 = false;

    boolean toggle1 = true;
    boolean toggle2 = true;
    boolean gateToggle1 = true;
    boolean gateToggle2 = false;

    boolean lastToggleA = true;
    boolean lastToggleX = true;
    boolean lastToggleDown = true;
    boolean lastToggleUp = true;
    boolean lastToggleY = true;

    public RiseTeleOp_FF_Control(String flName, String frName, String brName, String blName, String Arm1, String Arm2, String CageName, String GateName, String IntakeName, HardwareMap hardwareMap , Telemetry telemetry, Gamepad gamepad1){
        driveTrain = new MecanumDriveTrain(flName, frName, brName, blName, hardwareMap);
        arm = new Arm(Arm1, Arm2, CageName, hardwareMap, telemetry);
        gate = new Gate(GateName, hardwareMap);
        intake = new Intake(IntakeName, hardwareMap);

        driveTrain.setBreakMode();
        driveTrain.reset();

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
    }

    public void teleOpArm(){
        if ((gamepad1.x != lastToggleX) && gamepad1.x && (arm.ArmState == Arm.armState.HOME)){
            arm.ArmState = Arm.armState.TOP_LEVEL;
        }

        if ((gamepad1.x != lastToggleX) && gamepad1.x && (arm.ArmState == Arm.armState.TOP_LEVEL || arm.ArmState == Arm.armState.MIDDLE_LEVEL || arm.ArmState == Arm.armState.BOTTOM_LEVEL)){
            //        if ((gamepad1.x != lastToggleX) && gamepad1.x && (arm.ArmState != Arm.armState.HOME)){
            arm.ArmState = Arm.armState.HOME;
        }

        if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down && arm.ArmState == Arm.armState.TOP_LEVEL){
            arm.ArmState = Arm.armState.MIDDLE_LEVEL;
        }

        if ((gamepad1.dpad_down != lastToggleDown) && gamepad1.dpad_down && arm.ArmState == Arm.armState.MIDDLE_LEVEL){
            arm.ArmState = Arm.armState.BOTTOM_LEVEL;
        }

        if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up && arm.ArmState == Arm.armState.BOTTOM_LEVEL){
            arm.ArmState = Arm.armState.MIDDLE_LEVEL;
        }

        if ((gamepad1.dpad_up != lastToggleUp) && gamepad1.dpad_up && arm.ArmState == Arm.armState.MIDDLE_LEVEL){
            arm.ArmState = Arm.armState.TOP_LEVEL;
        }

        lastToggleX = gamepad1.x;
        lastToggleDown = gamepad1.dpad_down;
        lastToggleUp = gamepad1.dpad_up;
    }

    public void teleOpGate(){
        if ((gamepad1.y != lastToggleY) && gamepad1.y && gateToggle1) {
            gate.GateState = Gate.gateState.OPEN;
            gateToggle1 = false;
            gateToggle2 = true;

        } else if ((gamepad1.y != lastToggleY) && gamepad1.y && gateToggle2) {
            gate.GateState = Gate.gateState.CLOSED;
            gateToggle1 = true;
            gateToggle2 = false;
        }
        lastToggleY = gamepad1.y;
    }

    public void teleOpIntake(){
        intake.intakeMotor.setPower(Math.pow(gamepad1.right_trigger - gamepad1.left_trigger, 3));
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
}
