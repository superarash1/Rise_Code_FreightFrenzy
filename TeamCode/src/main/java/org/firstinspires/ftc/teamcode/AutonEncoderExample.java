package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutonEncoderExample extends LinearOpMode {

    Hardware TIseBot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    // FineTune
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 0.66; //Could be wrong; if things break, look at this
    static final double WHEEL_DIAMETER_MM = 96.0;

    //static final double COUNTS_PER_MM = ((((1+(46/17))) * (1+(46/11))) * 28); // Potentially use goBilda provided code
    static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415);

    // Add P-Drive
    static final double DRIVE_SPEED_1 = 0.6;

    static final double distance1  = 1000;

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


        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d", TIseBot.frontLeftMotor.getCurrentPosition(), TIseBot.frontRightMotor.getCurrentPosition(), TIseBot.backRightMotor.getCurrentPosition(), TIseBot.backLeftMotor.getCurrentPosition());
        telemetry.update();

        telemetry.update();

        waitForStart();

        encoderDrive(DRIVE_SPEED_1,  distance1, 5.0);  // S1: Forward 300 Millimeters with 5 Sec timeout

    }
    public void encoderDrive(double speed, double distanceMM, double timeoutS) {

        // Creates an array for the target positions for each of your motors
        int []newWheelTarget = new int[4];

        if (opModeIsActive()) {

            for (int i = 0; i < 4; i++) {
                // Defines the wheel target
                newWheelTarget[i] = wheels[0].getCurrentPosition() + (int)(distanceMM * COUNTS_PER_MM);

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
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newWheelTarget[0],  newWheelTarget[1], newWheelTarget[2], newWheelTarget[3]);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", TIseBot.frontLeftMotor.getCurrentPosition(), TIseBot.frontRightMotor.getCurrentPosition(), TIseBot.backRightMotor.getCurrentPosition(), TIseBot.backLeftMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stops all motion;
            TIseBot.frontLeftMotor.setPower(0);
            TIseBot.frontRightMotor.setPower(0);
            TIseBot.backRightMotor.setPower(0);
            TIseBot.backLeftMotor.setPower(0);


            // Turn off RUN_TO_POSITION
            TIseBot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            TIseBot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            TIseBot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            TIseBot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(50);   // optional pause after each move
        }
    }
}
