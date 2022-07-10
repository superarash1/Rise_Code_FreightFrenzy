package org.firstinspires.ftc.teamcode.OldFFCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.otherCode.VegaHardware;

@Disabled
public class legitBotAuton extends LinearOpMode {

    VegaHardware robot = new VegaHardware();

    private ElapsedTime runtime = new ElapsedTime();

    public double globalAngle;
    // Constants to find the amount of encoder ticks per CM
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.35;
    static final double WHEEL_DIAMETER_CM = 9.60;

    static final double ARM_COUNTS_PER_DEGREE = (537.7/360);

    // Finds the amount of encoder ticks per MM
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);


    // These are the distances each motor will go (negation means opposite direction, which is why each motor is separately defined)

    static final double fLeft1  = 100;
    static final double fRight1  = 100;
    static final double bRight1  = 100;
    static final double bLeft1  = 100;

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

        // Robot drives 100 cm forward (roughly)
        encoderDrive(0.5 , -23, -23, -23, -23, 5.0);
        sleep(50);
        encoderDrive(0.5 , 18, -18, -18, 18, 5.0);
        sleep(250);
        encoderDrive(0.5 , -18, -18, -18, -18, 5.0);
        sleep(250);

        timeBased(3);
    }

    public void armEncoder(double speed, double armDegrees) {

        // Creates an array for the target positions for each of your motors
        int armTarget;

        if (opModeIsActive()) {

            // Defines the target position for your motor
            armTarget = robot.arm.getCurrentPosition() + (int)(armDegrees);

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
            while (opModeIsActive() && (robot.arm.isBusy())) {
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

    public void timeBased(double time){
        runtime.reset();
        robot.intake.setPower(0.7);
        while (runtime.seconds() < time/2){

        }
        armEncoder(0.05, 1);

        runtime.reset();
        while (runtime.seconds() < time/2){

        }
        robot.intake.setPower(0);

    }

    public double calculateTotalAngle(){
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = angles.firstAngle - robot.straight.firstAngle;

        return -globalAngle;
    }

    public void encoderDrive(double speed, double fLeftMM, double fRightMM,double bRightMM,double bLeftMM, double timeoutS) {

        // Creates an array for the target positions for each of your motors
        int []newWheelTarget = new int[4];

        if (opModeIsActive()) {

            // Defines the target position for your motor
            newWheelTarget[0] = wheels[0].getCurrentPosition() + (int)(fLeftMM * COUNTS_PER_CM);
            newWheelTarget[1] = wheels[1].getCurrentPosition() + (int)(fRightMM * COUNTS_PER_CM);
            newWheelTarget[2] = wheels[2].getCurrentPosition() + (int)(bRightMM * COUNTS_PER_CM);
            newWheelTarget[3] = wheels[3].getCurrentPosition() + (int)(bLeftMM * COUNTS_PER_CM);

            for (int i = 0; i < 4; i++) {
                // Sets the target position for the motors
                wheels[i].setTargetPosition(newWheelTarget[i]);

                // Tells the motor to drive until they reach the target position
                wheels[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
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

            //  sleep(50);   // optional pause after each move
        }
    }
}