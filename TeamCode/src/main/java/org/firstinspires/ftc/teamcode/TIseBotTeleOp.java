package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class TIseBotTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware TIseBot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        double drive;
        double turn;
        double strafe;
        double fLeft;
        double fRight;
        double bLeft;
        double bRight;
        double mLeft;
        double mRight;
        double max;


        // Initialize the hardware variables using the init() method in the hardware class
        TIseBot.init(hardwareMap);

        // Set up the motor's default setting
        TIseBot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TIseBot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TIseBot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TIseBot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TIseBot.middleLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TIseBot.middleRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TIseBot.sliderLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Press Button to Activate Opmode");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){

            // Add robot data to telemetry
            telemetry.addData("Angle:", TIseBot.calculateAngle());
            telemetry.update();

            // Gamepad controls for movement
            drive = gamepad1.left_stick_y; //Between -1 and 1
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.dpad_left? -1:gamepad1.dpad_right? 1: gamepad1.left_stick_x;


            // Mechanum Drive Calculations
            fLeft = .8*drive - .8*strafe - .75*turn;
            fRight = .7*drive + .8*strafe + .75*turn;
            bRight = .7*drive - .8*strafe + .75*turn;
            bLeft = .7*drive + .8*strafe - .75*turn;
            mLeft = drive;
            mRight = drive;

            // This ensures that the power values the motors are set to are in the range (-1, 1)
            max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
            if (max > 1.0) {
                fLeft /= max;
                fRight /= max;
                bLeft /= max;
                bRight /= max;
                mLeft /= max;
                mRight /= max;
            }

            // Set power to the motors
            TIseBot.frontRightMotor.setPower(fRight);
            TIseBot.frontLeftMotor.setPower(fLeft);
            TIseBot.backRightMotor.setPower(bRight);
            TIseBot.backLeftMotor.setPower(bLeft);
            TIseBot.middleLeftMotor.setPower(mLeft);
            TIseBot.middleRightMotor.setPower(mRight);

            // Turns the "claw" door
            if(gamepad1.dpad_right) {
                TIseBot.claw.setPower(1);
            } else if(gamepad1.dpad_left) {
                TIseBot.claw.setPower(-1);
            } else {
                TIseBot.claw.setPower(0);
            }

            // Turns the "joint" up and down
            if (gamepad1.dpad_up) {
                TIseBot.joint.setPower(0.4);
            } else if (gamepad1.dpad_down) {
                TIseBot.joint.setPower(-0.4);
            } else {
                TIseBot.joint.setPower(0);
            }

            // Moves the linear sliders up and down
            if (gamepad1.right_trigger > 0) {
                TIseBot.sliderLift.setPower(0.15);
            } else if (gamepad1.left_trigger > 0) {
                TIseBot.sliderLift.setPower(-0.15);
            } else {
                TIseBot.sliderLift.setPower(0);
            }

            // Pause for 20 mS each cycle = update 50 times a second.
            sleep(20);
        }
    }
}
