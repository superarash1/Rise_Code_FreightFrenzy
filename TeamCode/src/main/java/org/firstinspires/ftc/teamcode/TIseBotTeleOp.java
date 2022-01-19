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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Mat;

@TeleOp
public class TIseBotTeleOp extends LinearOpMode {

    /* Declare OpMode members */
    Hardware TIseBot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    public double globalAngle;
    Orientation lastAngles = new Orientation();

    public double spinnerCM = 1;

    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double ARM_COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV / 360)*0.45;

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

        boolean gateToggle1 = true;
        boolean gateToggle2 = false;

        double coolDown = 50;

        Thread  driveThread = new DriveThread();

        // Initialize the hardware variables using the init() method in the hardware class
        TIseBot.init(hardwareMap);

        wheels[0] = TIseBot.frontLeftMotor;
        wheels[1] = TIseBot.frontRightMotor;
        wheels[2] = TIseBot.backRightMotor;
        wheels[3] = TIseBot.backLeftMotor;

        for (DcMotor wheel : wheels){
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

        TIseBot.arm1.setTargetPosition(1);
        TIseBot.arm2.setTargetPosition(1);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Press Button to Activate OpMode");
        telemetry.update();

        waitForStart();

        driveThread.start();

        // TODO: Potentially remove RUN_TO_POSITION or move that stuff inside and justre-intialize method
        // Maybe move this below

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){

            realAngle = TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE;

            // Gamepad controls for movement
            //TODO: Have Abdullah test and test yourself
            drive = Math.pow(gamepad1.left_stick_y, 3); //Between -1 and 1
            turn = Math.pow(gamepad1.right_stick_x, 3);
            strafe = Math.pow(gamepad1.left_stick_x, 3);

//            drive = gamepad1.left_stick_y;
//            turn = gamepad1.right_stick_x;
//            strafe = gamepad1.left_stick_x;


            // Mechanum Drive Calculations
            fLeft = -.7*drive + .8*strafe + .75*turn;
            fRight = -.7*drive - .8*strafe - .75*turn;
            bRight = -.7*drive + .8*strafe - .75*turn;
            bLeft = -.7*drive - .8*strafe + .75*turn;

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
            TIseBot.backRightMotor.setPower((bRight)*1.5);
            TIseBot.backLeftMotor.setPower((bLeft)*1.5);


//            if (gamepad1.left_bumper) {
//                if (open) {
//                    TIseBot.boxO.setPosition(0.2083333);
//                    open = false;
//                }
//                else if (open = false) {
//                    TIseBot.boxO.setPosition(0);
//                    open = true;
//                }
//            }
//            if (gamepad1.right_bumper) {
//                TIseBot.boxT.setPosition(70/360);
//            }
//            if (gamepad1.right_bumper) {
//                TIseBot.duck1.setPosition(TIseBot.duck1.getPosition() + 1);
//            }
//            if (gamepad1.left_bumper) {
//                TIseBot.duck2.setPosition(TIseBot.duck2.getPosition() + 1);
//            }
//
//            if (duckSpin <= 10) {
//                duckSpin = 0;
//            }
//
//            telemetry.addData("Power of duck1:", -duckSpin);
//            telemetry.addData("Power of duck2:", duckSpin);

            TIseBot.intake.setPower((gamepad1.right_trigger - gamepad1.left_trigger)*.2);

            if (gamepad1.dpad_right){
                TIseBot.cageSpin1.setPosition(TIseBot.cageSpin1.getPosition() + 0.02);
                TIseBot.cageSpin2.setPosition(TIseBot.cageSpin2.getPosition() + 0.02);
            }

            if (gamepad1.dpad_left){
                TIseBot.cageSpin1.setPosition(TIseBot.cageSpin1.getPosition() - 0.02);
                TIseBot.cageSpin2.setPosition(TIseBot.cageSpin2.getPosition() - 0.02);
            }

            if (gamepad1.b && gateToggle1 && coolDown > 10){
                TIseBot.gate.setPosition(TIseBot.gate.getPosition() + 0.5);
                gateToggle1 = false;
                gateToggle2 = true;
            }

            if (gamepad1.b && gateToggle2 && coolDown > 10){
                TIseBot.gate.setPosition(TIseBot.gate.getPosition() - 0.5);
                gateToggle2 = false;
                gateToggle1 = true;
            }


            if (gamepad1.dpad_up && TIseBot.arm1.getTargetPosition() == 290){
                TIseBot.arm1.setTargetPosition(315);
                TIseBot.arm2.setTargetPosition(315);
            }

            if (gamepad1.dpad_up && TIseBot.arm1.getTargetPosition() == 315){
                TIseBot.arm1.setTargetPosition(330);
                TIseBot.arm2.setTargetPosition(330);
            }

            if (gamepad1.dpad_down && TIseBot.arm1.getTargetPosition() == 330){
                TIseBot.arm1.setTargetPosition(315);
                TIseBot.arm2.setTargetPosition(315);
            }

            if (gamepad1.dpad_down && TIseBot.arm1.getTargetPosition() == 315){
                TIseBot.arm1.setTargetPosition(290);
                TIseBot.arm2.setTargetPosition(290);
            }

            if (gamepad1.right_bumper){
                TIseBot.carouselSpinner.setPower(-0.3);
            } else{
                TIseBot.carouselSpinner.setPower(0);
            }

            coolDown++;

            // Pause for 20 mS each cycle = update 50 times a second.
            sleep(20);
        }

        TIseBot.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveThread.interrupt();
    }
    // Finds the current angle of the robot relative to its initial position
    public double calculateAngle(){ // TODO: Check axes order in teleOp and test with radians for fun
        Orientation angles = TIseBot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle - TIseBot.straight.firstAngle;
    }

    private void resetAngle() {
        lastAngles = TIseBot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = TIseBot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    // TODO: Code Carousel Spinner
    public void carouselSpinner(){

    }

    public void armSpinner(){
        double kp = 0.002;
        double kd = 0.0016; // 0.001
        double ki = 0.1;

        double kGravity = 0.04; //0.04

        double gravity = 0;
        double power = 0;

        double errorChange;
        double P = 0;

        double Derivative = 0;
        double previousDerivative = 0;

        double previousDerivativeSecond = 0;
        double derivativeSecond = 0;

        double maxIntegralSum = 0.05;

        double previousError = 0;
        double error = 0;

        double area = 0;
        double previousArea = 0;

        double dt = 50;
        double dts = dt/1000;

        double a = 0.5; // a can be anything from 0 < a < 1
        double previousFilterEstimate = 0;
        double currentFilterEstimate = 0;

        double coolDown = 100;

        boolean toggle1 = true;
        boolean toggle2 = false;

        double firstError = 0;

        while (opModeIsActive()) {
            telemetry.addData("Target Angle: ", TIseBot.arm1.getTargetPosition()*ARM_COUNTS_PER_DEGREE - 45);
            telemetry.addData("Current Angle: ", TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE - 45);

            telemetry.addData("Gravity", kGravity*gravity);

            telemetry.addData("dtS", runtime.seconds());

            telemetry.addData("Power", power);
            telemetry.addData("Derivative: ",  (kd * (currentFilterEstimate/dts)));

            telemetry.addData("Error: ", error);

            telemetry.addData("Proportion: ", error*kp);

            telemetry.addData("Integral: ", ki * area);

            telemetry.addData("Previous Error: ", previousError);

            telemetry.addData("de(t)/dt: ", (error - previousError)/dts);


            telemetry.addData("Area: ", area);
            telemetry.addData("Previous Area: ", previousArea);

            telemetry.update();


            if (gamepad1.x && toggle1 && coolDown > 100 && error < 50){
                TIseBot.arm1.setTargetPosition(5);
                TIseBot.arm2.setTargetPosition(5);
                toggle1 = false;
                toggle2 = true;
            } else if (gamepad1.x && toggle2 && coolDown > 100 && error < 50){
                TIseBot.arm1.setTargetPosition(290);
                TIseBot.arm2.setTargetPosition(290);
                toggle1 = true;
                toggle2 = false;
            }

            previousError = error;

            error = TIseBot.arm1.getTargetPosition()*ARM_COUNTS_PER_DEGREE - realAngle;

//            P = error/Math.abs(TIseBot.arm1.getTargetPosition()*ARM_COUNTS_PER_DEGREE - 49);
            errorChange = error - previousError;

            previousFilterEstimate = currentFilterEstimate;
            currentFilterEstimate = (1-a) * errorChange + (a * previousFilterEstimate);

            derivativeSecond = (Derivative - previousDerivative)/dts;

            previousArea = area;

            if (area*previousArea < 0) previousArea = 0;

            area = (error * dts) + previousArea;

//            maxIntegralSum = Math.cos((TIseBot.arm1.getTargetPosition()*ARM_COUNTS_PER_DEGREE - 49)*(Math.PI/180));

            if (area > maxIntegralSum) {
                area = maxIntegralSum;
            }

            if (area < -maxIntegralSum) {
                area = -maxIntegralSum;
            }

            gravity = Math.cos((realAngle - 49)*(Math.PI/180));

            //power = kp * P + kd * ((error - previousError)/dts) + ki * area + (Math.signum(error)*kGravity*gravity);
            power = kp * error + kd * (currentFilterEstimate/dts) + ki * area + (Math.signum(error)*kGravity*gravity);

            //power = kp * error + kd * Derivative + kd2 * derivativeSecond + ki * area + (Math.signum(error)*kGravity*gravity);

            TIseBot.arm1.setPower(-power);
            TIseBot.arm2.setPower(power);

            coolDown++;

            sleep((long) dt);
        }
        TIseBot.arm1.setTargetPosition(0);
        TIseBot.arm2.setTargetPosition(0);
    }

    private class DriveThread extends Thread {
        public DriveThread() {
            this.setName("DriveThread");

            telemetry.addData("%s", this.getName());
        }
        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {
            telemetry.addData("Starting thread %s",this.getName());
            telemetry.update();
            while (!isInterrupted()) {
                armSpinner();
            }

            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            // an error occurred in the run loop.
            telemetry.addData("end of thread %s", this.getName());
        }
    }
}