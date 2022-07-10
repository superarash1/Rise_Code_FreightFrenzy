package org.firstinspires.ftc.teamcode.AutonExamplesOld;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OldFFCode.Hardware;

@Disabled
public class AutonEncoderExample extends LinearOpMode {

    Hardware TIseBot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    // Constants to find the amount of encoder ticks per CM
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.35;
    static final double WHEEL_DIAMETER_CM = 9.60;

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
        TIseBot.init(hardwareMap);

        // assigning each element in the array to a specific motor
        wheels[0] = TIseBot.frontLeftMotor;
        wheels[1] = TIseBot.frontRightMotor;
        wheels[2] = TIseBot.backRightMotor;
        wheels[3] = TIseBot.backLeftMotor;

        // Set up for each motor
        for (DcMotor wheel : wheels){
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Telemetry to show initial encoder position
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d", TIseBot.frontLeftMotor.getCurrentPosition(), TIseBot.frontRightMotor.getCurrentPosition(), TIseBot.backRightMotor.getCurrentPosition(), TIseBot.backLeftMotor.getCurrentPosition());
        telemetry.update();

        telemetry.update();

        waitForStart();

        // Robot drives 100 cm forward (roughly)
        encoderDrive(0.6 , fLeft1, fRight1, bRight1, bLeft1, 5.0);
        sleep(250);

        // Robot Turns roughly 90 degrees tho everything is measured in mm
        encoderDrive(0.6, fLeft2, fRight2, bRight2, bLeft2, 5.0);
        sleep(250);

        // Robot strafes back to start
        encoderDrive(0.6, fLeft3, fRight3, bRight3, bLeft3, 5.0);
        sleep(250);
    }

    public void armEncoder(double speed, double armDegree){


    }

    public void encoderDrive(double speed, double fLeftMM, double fRightMM,double bRightMM,double bLeftMM, double timeoutS) {

        // Creates an array for the target positions for each of your motors
        int []newWheelTarget = new int[4];

        if (opModeIsActive()) {

            // Defines the target position for your motor
            newWheelTarget[0] = wheels[0].getCurrentPosition() + (int)(fLeftMM / COUNTS_PER_CM);
            newWheelTarget[1] = wheels[1].getCurrentPosition() + (int)(fRightMM / COUNTS_PER_CM);
            newWheelTarget[2] = wheels[2].getCurrentPosition() + (int)(bRightMM / COUNTS_PER_CM);
            newWheelTarget[3] = wheels[3].getCurrentPosition() + (int)(bLeftMM / COUNTS_PER_CM);

            for (int i = 0; i < 4; i++) {
                // Sets the target position for the motors
                wheels[i].setTargetPosition(newWheelTarget[i]);

                // Tells the motor to drive until they reach the target position
                wheels[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            // reset the timeout time and start motion.
            runtime.reset();

            // Sets speed (negation indicates direction;
            // in this case negative is forwards because that's how the motor orientation was set up in the Hardware map and I don't wanna change all of my code to make forward positive)
            TIseBot.frontLeftMotor.setPower(-speed);
            TIseBot.frontRightMotor.setPower(-speed);
            TIseBot.backRightMotor.setPower(-speed);
            TIseBot.backLeftMotor.setPower(-speed);

            // Will keep looping until one of the of the motors reach the target position
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (TIseBot.frontLeftMotor.isBusy() && TIseBot.frontRightMotor.isBusy() && TIseBot.backRightMotor.isBusy() && TIseBot.backLeftMotor.isBusy())) {
                // Displays the target position and current position in telemetry
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newWheelTarget[0],  newWheelTarget[1], newWheelTarget[2], newWheelTarget[3]);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", TIseBot.frontLeftMotor.getCurrentPosition(), TIseBot.frontRightMotor.getCurrentPosition(), TIseBot.backRightMotor.getCurrentPosition(), TIseBot.backLeftMotor.getCurrentPosition());
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
