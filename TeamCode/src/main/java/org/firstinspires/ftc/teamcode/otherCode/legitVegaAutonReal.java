package org.firstinspires.ftc.teamcode.otherCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
public class legitVegaAutonReal extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    VegaHardware robot = new VegaHardware();
    // Constants to find the amount of encoder ticks per CM
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCH = 3.77953;

    static final double ARM_COUNTS_PER_DEGREE = (537.7/360);

    Orientation lastAngles = new Orientation();
    public double globalAngle;

    // Finds the amount of encoder ticks per MM
    static final double WHEEL_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH * Math.PI);

    // These are the distances each motor will go (negation means opposite direction, which is why each motor is separately defined)

    static final double fLeft1  = 23;
    static final double fRight1  = 23;
    static final double bRight1  = 23;
    static final double bLeft1  = 23;

    static final double fLeft2  = -57.5;
    static final double fRight2  = 57.5;
    static final double bRight2  = 57.5;
    static final double bLeft2  = -57.5;

    static final double fLeft3  = 100;
    static final double fRight3  = -100;
    static final double bRight3  = 100;
    static final double bLeft3  = -100;

    // Creating an array for the wheels (makes it more organized)
    DcMotor[] wheels = new DcMotor[4];

    @Override
    public void runOpMode() {


        // Initializing Hardware

        robot.init(hardwareMap);

        // assigning each element in the array to a specific motor
        wheels[0] = robot.frontLeft;
        wheels[1] = robot.frontRight;
        wheels[2] = robot.backRight;
        wheels[3] = robot.backLeft;


        wheels[0].setDirection(DcMotor.Direction.FORWARD);
        wheels[1].setDirection(DcMotor.Direction.FORWARD);
        wheels[2].setDirection(DcMotor.Direction.FORWARD);
        wheels[3].setDirection(DcMotor.Direction.FORWARD);

        // Set up for each motor
        for (DcMotor wheel : wheels){
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Telemetry to show initial encoder position
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d", robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(), robot.backRight.getCurrentPosition(), robot.backLeft.getCurrentPosition());
        telemetry.update();

        telemetry.update();

        waitForStart();

        PIDDrive(23, 1);
        // Robot drives 100 cm forward (roughly)
//        encoderDrive(0.3 , -fLeft1, -fRight1, -bRight1, -bLeft1, 5.0);
//        sleep(50);
////        IMU_Turn_PID_Total(45, 1);
//        sleep(50);
//        encoderDrive(0.5 , -3, -3, -3, -3, 5.0);
//        sleep(50);
//        timeBased(3);
//        sleep(50);
//
////        IMU_Turn_PID_Total(0, 1);
//
//        encoderDrive(0.5 , 23, 23, 23, 23, 5.0);
//
//        encoderDrive(0.5 , 26, -26, 26, -26, 5.0);
//
//        runtime.reset();
//        while (runtime.seconds() < 5){
//            robot.spinner.setPower(1);
//        }
//
//        encoderDrive(0.5 , -24, -24, -24, -24, 5.0);
//        sleep(50);
//
//        encoderDrive(0.5 , 5, -5, 5, -5, 5.0);
//        sleep(50);

        sleep(250);
    }

    public void armEncoder(double speed, double armDegrees) {

        // Creates an array for the target positions for each of your motors
        int armTarget;

        if (opModeIsActive()) {

            // Defines the target position for your motor
            armTarget = robot.arm.getCurrentPosition() + (int)(armDegrees / ARM_COUNTS_PER_DEGREE);

            for (int i = 0; i < 4; i++) {
                // Sets the target position for the motors
                robot.arm.setTargetPosition(armTarget);

                // Tells the motor to drive until they reach the target position
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            // reset the timeout time and start motion.

            // Sets speed (negation indicates direction;
            // in this case negative is forwards because that's how the motor orientation was set up in the Hardware map and I don't wanna change all of my code to make forward positive)
            robot.arm.setPower(speed);

            // Will keep looping until one of the of the motors reach the target position
            while (opModeIsActive() && (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy() && robot.backLeft.isBusy())) {
                // Displays the target position and current position in telemetry
                telemetry.addData("Path1, Running to:", armTarget);
                telemetry.addData("Path2, Running at %7d :%7d :%7d :%7d", robot.arm.getCurrentPosition());
                telemetry.update();
            }

            robot.arm.setPower(0);
            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderDrive(double speed, double fLeftMM, double fRightMM,double bRightMM,double bLeftMM, double timeoutS) {

        // Creates an array for the target positions for each of your motors
        int []newWheelTarget = new int[4];

        if (opModeIsActive()) {

            // Defines the target position for your motor
            newWheelTarget[0] = (int)(-fLeftMM * WHEEL_COUNTS_PER_INCH);
            newWheelTarget[1] = (int)(-fRightMM * WHEEL_COUNTS_PER_INCH);
            newWheelTarget[2] = (int)(-bRightMM * WHEEL_COUNTS_PER_INCH);
            newWheelTarget[3] = (int)(-bLeftMM * WHEEL_COUNTS_PER_INCH);

            robot.frontLeft.setTargetPosition(newWheelTarget[0]);
            robot.frontRight.setTargetPosition(newWheelTarget[1]);
            robot.backRight.setTargetPosition(newWheelTarget[2]);
            robot.backLeft.setTargetPosition(newWheelTarget[3]);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//            for (int i = 0; i < 4; i++) {
//                // Sets the target position for the motors
//                wheels[i].setTargetPosition(newWheelTarget[i]);
//
//                // Tells the motor to drive until they reach the target position
//                wheels[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
            // reset the timeout time and start motion.

            // Sets speed (negation indicates direction;
            // in this case negative is forwards because that's how the motor orientation was set up in the Hardware map and I don't wanna change all of my code to make forward positive)
            robot.frontLeft.setPower(-speed);
            robot.frontRight.setPower(-speed);
            robot.backRight.setPower(-speed);
            robot.backLeft.setPower(-speed);

            // Will keep looping until one of the of the motors reach the target position
            while (opModeIsActive() && (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy() && robot.backLeft.isBusy())) {
                // Displays the target position and current position in telemetry
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newWheelTarget[0],  newWheelTarget[1], newWheelTarget[2], newWheelTarget[3]);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(), robot.backRight.getCurrentPosition(), robot.backLeft.getCurrentPosition());
                telemetry.update();
            }

            for (DcMotor wheel : wheels){
                // Stops motors after motors have reached target position
                wheel.setPower(0);
                // Resets encoders
                wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void PIDDrive(double distance, double tolerance) { // TODO: Adjust Tolerance

//        double []newWheelTarget = new double[4];

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
//            for (int i = 0; i < 4; i++) {
//                newWheelTarget[i] = wheels[i].getCurrentPosition()/WHEEL_COUNTS_PER_INCH + (distance); // TODO: Get avg position
//            }

            double kp;
            double kd; // Constant of derivation
            double ki;

            double kStatic;
            double kV;

            double a; // a can be anything from 0 < a < 1     0.55

            double previousFilterEstimate = 0;
            double currentFilterEstimate = 0;

            double referenceVelocity;

            double dt; //Delta time = 20 ms/cycle
            double dtS;

            double PID = 0;

            double power = 0;

            double P = 0;

            double previousError = 0;
            double error = tolerance + 1;

            double area = 0;
            double previousArea = 0;

            kp = 1;
            kd = 0; // Constant of derivation
            ki = 0;

            kV = 0;
            kStatic = 0;

            referenceVelocity = 0;

            dt = 50; //Delta time = 20 ms/cycle
            dtS = dt/1000;

            error = tolerance + 1;

            while ((Math.abs(error) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError

                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", distance, distance, distance, distance);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", wheels[0].getCurrentPosition(), wheels[1].getCurrentPosition(), wheels[2].getCurrentPosition(), wheels[3].getCurrentPosition());

//                telemetry.addData("DistanceCM: ", distance);

                telemetry.addData("power: ", power);
//
//                telemetry.addData("Friction FeedForward:", kStatic * Math.signum(PID));
//                telemetry.addData("Velocity FeedForward:", kV * (referenceVelocity * Math.signum(PID)));
//
//                telemetry.addData("Proportion:", P);
//                telemetry.addData("Derivative:", kd * ((error - previousError) / dtS));
//                telemetry.addData("Integral:", ki * area);
////
////                telemetry.addData("de(t)/dt", ((error - previousError) / dtS));
////
////                telemetry.addData("error:", error);
////                telemetry.addData("previous error:", previousError);
////
////                telemetry.addData("∫e(t)dt:", area);
////                telemetry.addData("previous ∫e(t)dt:", previousArea);
////
////                telemetry.addData("dtS", dtS);

                telemetry.update();

                previousError = error;

                error = distance - wheels[0].getCurrentPosition()/WHEEL_COUNTS_PER_INCH;

                P = Math.abs(error)/distance;

                previousArea = area;

                if (error*(int)(distance) < 0) previousArea = 0;

                area = error * dtS + previousArea;

                PID = kp * P + kd * ((error - previousError) / dtS) + (ki * area);

//                power = PID + kStatic * Math.signum(PID) + kV * (referenceVelocity * Math.signum(PID));

                wheels[0].setPower(Math.abs(error)/(int)distance);
                wheels[1].setPower(Math.abs(error)/(int)distance);
                wheels[2].setPower(Math.abs(error)/(int)distance);
                wheels[3].setPower(Math.abs(error)/(int)distance);

                sleep((long) 50);
            }

            // Stop all motion;
            for (int i = 0; i < 4; i++){
                wheels[i].setPower(0);

                // Resets encoders
                wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                wheels[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public double calculateTotalAngle(){
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = angles.firstAngle - robot.straight.firstAngle;

        return -globalAngle;
    }

    public void timeBased(double time){
        runtime.reset();
        robot.slide.setPower(0.7);
        while (runtime.seconds() < time/2){

        }
        armEncoder(0.35, 180);

        runtime.reset();
        while (runtime.seconds() < time/2){

        }
        robot.slide.setPower(0);

    }

    public void IMU_Turn_PID_Total(double degree, double tolerance) {

        double totalDistance = degree - calculateTotalAngle();

        double kp = 1;
        double kd = 0;
        double ki = 0;

        double power = 0;

        double P = 0;

        double previousError = 0;
        double error = tolerance + 1;

        double area = 0;
        double previousArea = 0;

        double dt = 20;
        double dts = dt/1000;

        while (opModeIsActive() && (Math.abs(error) > tolerance)) {

            telemetry.addData("Target Angle: ", degree);
            telemetry.addData("Current Angle: ", calculateTotalAngle());

            telemetry.addData("Total Distance", totalDistance);

            telemetry.addData("Power", power);

            telemetry.addData("de(t)/dt: ", (error - previousError)/dts);

            telemetry.addData("Proportion: ", P);

            telemetry.addData("Error: ", error);

            telemetry.addData("Derivative: ", kd * ((error - previousError)/dts));
            telemetry.addData("Integral: ", ki * area);


            telemetry.addData("Previous Error: ", previousError);

            telemetry.addData("Area: ", area);
            telemetry.addData("Previous Area: ", previousArea);

            telemetry.addData("dtS", dts);

            telemetry.update();

            previousError = error;
            error = degree - calculateTotalAngle();

            previousArea = area;
            area = (error * dts) + previousArea;

            P = Math.abs(error)/totalDistance;

            power = kp * P + kd * ((error - previousError)/dts) + ki * area;

            wheels[0].setPower(-power);
            wheels[1].setPower(power);
            wheels[2].setPower(power);
            wheels[3].setPower(-power);

            sleep((long) dt);
        }

        for (int i = 0; i < 4; i++){
            wheels[i].setPower(0);
        }
    }
}
