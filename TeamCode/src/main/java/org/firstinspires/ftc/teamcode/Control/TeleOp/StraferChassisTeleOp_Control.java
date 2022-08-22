package org.firstinspires.ftc.teamcode.Control.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.MecanumDriveTrain;

public class StraferChassisTeleOp_Control {
    MecanumDriveTrain driveTrain;

    Gamepad gamepad1;

    double power;

    double drive;
    double turn;
    double strafe;
    double fLeft;
    double fRight;
    double bLeft;
    double bRight;
    double max;

    public StraferChassisTeleOp_Control(String flName, String frName, String brName, String blName, HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry){
        driveTrain = new MecanumDriveTrain(flName, frName, brName, blName, hardwareMap);

        driveTrain.setBreakMode();
        driveTrain.reset();

        this.gamepad1 = gamepad1;
    }

    public void Drive(){
        drive = Math.pow(gamepad1.left_stick_y, 3); //Between -1 and 1
        turn = Math.pow(gamepad1.right_stick_x, 3);
        strafe = Math.pow(gamepad1.left_stick_x, 3);

        // Mecanum Drive Calculations
        fLeft = -0.875 * drive + 1 * strafe + 0.8 * turn;
        fRight = -0.875 * drive - 1 * strafe - 0.8 * turn;
        bRight = -0.875 * drive + 1 * strafe - 0.8 * turn;
        bLeft = -0.875 * drive - 1 * strafe + 0.8 * turn;

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
