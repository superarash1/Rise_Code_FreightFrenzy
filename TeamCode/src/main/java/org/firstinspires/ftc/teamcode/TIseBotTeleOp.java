package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Mat;
import org.outoftheboxrobotics.neutrinoi2c.Rev2mDistanceSensor.AsyncRev2MSensor;

@TeleOp
public class TIseBotTeleOp extends LinearOpMode {

    /* Declare OpMode members */
    Hardware TIseBot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    public double globalAngle;
    Orientation lastAngles = new Orientation();

    public double firstError;

    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double ARM_COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV / 360) * 0.48;

    public double realAngle;

    DcMotor[] wheels = new DcMotor[4];

    @Override
    public void runOpMode() {

        double drive;
        double turn;
        double strafe;
        double fLeft;
        double fRight;
        double bLeft;
        double bRight;
        double max;

        double duckPower;

        boolean lastToggleY = true;
        double lastToggle2;

        double kp = 0.03; // 0.029  0.1
        double kd = 0.000105; // 0.00105
        double ki = 0; // 0.0011   0.00005

        double kGravity = 0.01; // 0.00475  0.005   0.025

        double a = 0.6; // a can be anything from 0 < a < 1     0.55

        double gravity = 0;
        double power = 0;

        double errorChange;
        double P = 0;

        double maxIntegralSum = 250;

        double previousError = 0;
        double error = 0;

        double area = 0;
        double previousArea = 0;

        double dt = 1;
        double dts = dt / 1000;

        double previousFilterEstimate = 0;
        double currentFilterEstimate = 0;

        double coolDown = 200;

        boolean toggle1 = true;
        boolean toggle2 = false;

        boolean gateToggle1 = true;
        boolean gateToggle2 = false;

        boolean autoToggle1 = true;
        boolean autoToggle2 = false;

//        double coolDown = 50;

//        Thread driveThread = new DriveThread();

        // Initialize the hardware variables using the init() method in the hardware class
        TIseBot.init(hardwareMap);

//        AsyncRev2MSensor asyncSensor = new AsyncRev2MSensor(TIseBot.sensor);

        wheels[0] = TIseBot.frontLeftMotor;
        wheels[1] = TIseBot.frontRightMotor;
        wheels[2] = TIseBot.backRightMotor;
        wheels[3] = TIseBot.backLeftMotor;

        for (DcMotor wheel : wheels) {
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


        TIseBot.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TIseBot.arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TIseBot.arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        TIseBot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TIseBot.arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TIseBot.arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        TIseBot.arm1.setTargetPosition(5);
        TIseBot.arm2.setTargetPosition(5);

//        TIseBot.arm1.setPositionPIDFCoefficients(1);
        firstError = (TIseBot.arm1.getTargetPosition() * ARM_COUNTS_PER_DEGREE - 44) - realAngle;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Press Button to Activate OpMode");
        telemetry.update();

        waitForStart();

//        driveThread.start();

        // TODO: Potentially remove RUN_TO_POSITION or move that stuff inside and justre-intialize method
        // Maybe move this below

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("cooldown", coolDown);

            telemetry.update();
            realAngle = TIseBot.arm1.getCurrentPosition() * ARM_COUNTS_PER_DEGREE;

            // Gamepad controls for movement
            //TODO: Have Abdullah test and test yourself
            drive = Math.pow(gamepad1.left_stick_y, 3); //Between -1 and 1
            turn = Math.pow(gamepad1.right_stick_x, 3);
            strafe = Math.pow(gamepad1.left_stick_x, 3);

            // Mechanum Drive Calculations
            fLeft = .7 * drive - .8 * strafe + .75 * turn;
            fRight = .7 * drive + .8 * strafe - .75 * turn;
            bRight = .7 * drive - .8 * strafe - .75 * turn;
            bLeft = .7 * drive + .8 * strafe + .75 * turn;

            // This ensures that the power values the motors are set to are in the range (-1, 1)
            max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
            if (max > 1.0) {
                fLeft /= max;
                fRight /= max;
                bLeft /= max;
                bRight /= max;
            }

            // Set power to the motors
            TIseBot.frontRightMotor.setPower(fRight);
            TIseBot.frontLeftMotor.setPower(fLeft);
            TIseBot.backRightMotor.setPower(bRight);
            TIseBot.backLeftMotor.setPower(bLeft);

            kGravity = 0;
            TIseBot.arm1.setPower( 0.2*(Math.pow(gamepad1.right_trigger - gamepad1.left_trigger, 3) + kGravity * gravity));
            TIseBot.arm2.setPower(-0.2*(Math.pow((gamepad1.right_trigger - gamepad1.left_trigger), 3) + kGravity * gravity)); // check negatived

            if (gamepad1.dpad_up) {
                TIseBot.cageSpin1.setPower(-0.3);

            } else if (gamepad1.dpad_down) {
                TIseBot.cageSpin1.setPower(0.3);

            } else {
                TIseBot.cageSpin1.setPower(0);

            }


            if (gamepad1.a) {
                TIseBot.cSpinRight.setPower(-0.6);
            } else {
                TIseBot.cSpinRight.setPower(0);
            }

            if ((gamepad1.y != lastToggleY) && gamepad1.y && gateToggle1) {
                TIseBot.gate.setPosition(0.2);
                gateToggle1 = false;
                coolDown = 0;
                gateToggle2 = true;

            } else if ((gamepad1.y != lastToggleY) && gamepad1.y && gateToggle2) {
                TIseBot.gate.setPosition(0);
                gateToggle1 = true;
                coolDown = 0;
                gateToggle2 = false;

            }

            lastToggleY = gamepad1.y;

            if (gamepad1.dpad_right){
                TIseBot.capper.setPower(0.25);
            } else if (gamepad1.dpad_left) {
                TIseBot.capper.setPower(-0.25);
            } else {
                TIseBot.capper.setPower(0);
            }

            coolDown++;

//            telemetry.addData("Distance", asyncSensor.getDistance(DistanceUnit.CM));
//            telemetry.addData("Last Reading", asyncSensor.getLastMeasurementTimestamp());

            if (gamepad1.right_bumper) {
                TIseBot.intake.setPower(0.3);

            } else if (gamepad1.left_bumper) {
                TIseBot.intake.setPower(-0.3);
            } else {
                TIseBot.intake.setPower(0);
            }


            // Pause for 20 mS each cycle = update 50 times a second.
            sleep(20);

        }
    }
}