package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class PID_Practice extends LinearOpMode {

    Hardware TIseBot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    // Constants to find the amount of encoder ticks per CM
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.35;
    static final double WHEEL_DIAMETER_CM = 9.60;

    // Finds the amount of encoder ticks per CM
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    static final double distance = 75;

    DcMotor[] wheels = new DcMotor[4];

    @Override
    public void runOpMode() {

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

        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d", TIseBot.frontLeftMotor.getCurrentPosition(), TIseBot.frontRightMotor.getCurrentPosition(), TIseBot.backRightMotor.getCurrentPosition(), TIseBot.backLeftMotor.getCurrentPosition());

        telemetry.update();

        waitForStart();

        PIDDrive(100, 1);
        sleep(100);

        PIDTurn(59, 1);
        sleep(100);

        PIDStrafe(-116, 1);
        sleep(100);

        PIDTurn(-59, 1);


    }

    public void PIDDrive(double distanceCM, double tolerance) { // TODO: Adjust Tolerance

        int []newWheelTarget = new int[4];

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            for (int i = 0; i < 4; i++) {
                newWheelTarget[i] = wheels[0].getCurrentPosition() + (int)(distanceCM * COUNTS_PER_CM); // TODO: Get avg position
            }

            double[] P = new double[4];

            double kp = 1;
            double kd = 0.000007; // Constant of derivation
            double ki = 0.00002;

            double dt = 20; //Delta time = 20 ms/cycle
            double dtS = dt/1000;

            double[] error = new double[4];
            double[] previousError = new double[4];
            double[] power = new double[4];

            double[] area = new double[4];
            double[] previousArea = new double[4];

            error[0] = tolerance + 1;

            while ((Math.abs(error[0]) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError

                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newWheelTarget[0], newWheelTarget[1], newWheelTarget[2], newWheelTarget[3]);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", wheels[0].getCurrentPosition(), wheels[1].getCurrentPosition(), wheels[2].getCurrentPosition(), wheels[3].getCurrentPosition());


                telemetry.addData("DistanceMM: ", (int)(distanceCM * COUNTS_PER_CM));

                telemetry.addData("power: ", power[0]);

                telemetry.addData("Proportion:", P[0]);
                telemetry.addData("Derivative:", kd * ((error[0] - previousError[0]) / dtS));
                telemetry.addData("Integral:", ki * area[0]);

                telemetry.addData("de(t)/dt", ((error[0] - previousError[0]) / dtS));

                telemetry.addData("error:", error[0]);
                telemetry.addData("previous error:", previousError[0]);

                telemetry.addData("∫e(t)dt:", area[0]);
                telemetry.addData("previous ∫e(t)dt:", previousArea[0]);

                telemetry.addData("dtS", dtS);

                telemetry.update();

                previousError[0] = error[0];
                previousError[1] = error[1];
                previousError[2] = error[2];
                previousError[3] = error[3];

                error[0] = (int)(distanceCM * COUNTS_PER_CM) - wheels[0].getCurrentPosition();
                error[1] = (int)(distanceCM * COUNTS_PER_CM) - wheels[1].getCurrentPosition();
                error[2] = (int)(distanceCM * COUNTS_PER_CM) - wheels[2].getCurrentPosition();
                error[3] = (int)(distanceCM * COUNTS_PER_CM) - wheels[3].getCurrentPosition();

                P[0] = Math.abs(error[0])/(int)(distanceCM * COUNTS_PER_CM);
                P[1] = Math.abs(error[1])/(int)(distanceCM * COUNTS_PER_CM);
                P[2] = Math.abs(error[2])/(int)(distanceCM * COUNTS_PER_CM);
                P[3] = Math.abs(error[3])/(int)(distanceCM * COUNTS_PER_CM);

                previousArea[0] = area[0];
                previousArea[1] = area[1];
                previousArea[2] = area[2];
                previousArea[3] = area[3];

                area[0] = error[0] * dtS + previousArea[0];
                area[1] = error[1] * dtS + previousArea[1];
                area[2] = error[2] * dtS + previousArea[2];
                area[3] = error[3] * dtS + previousArea[3];

                power[0] = kp * P[0] + kd * ((error[0] - previousError[0]) / dtS) + (ki * area[0]);
                power[1] = kp * P[1] + kd * ((error[1] - previousError[1]) / dtS) + (ki * area[1]);
                power[2] = kp * P[2] + kd * ((error[2] - previousError[2]) / dtS) + (ki * area[2]);
                power[3] = kp * P[3] + kd * ((error[3] - previousError[3]) / dtS) + (ki * area[3]);

                wheels[0].setPower(power[0]);
                wheels[1].setPower(power[1]);
                wheels[2].setPower(power[2]);
                wheels[3].setPower(power[3]);

                sleep((long) dt);
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


    public void PIDTurn(double distanceCM, double tolerance) { // TODO: Adjust Tolerance

        int []newWheelTarget = new int[4];

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            for (int i = 0; i < 4; i++) {
                newWheelTarget[i] = wheels[0].getCurrentPosition() + (int)(distanceCM * COUNTS_PER_CM); // TODO: Get avg position
            }

            double kp = 1;
            double kd = 0.0000002; // Constant of derivation
            double ki = 0.000003;

            double dt = 20; //Delta time = 20 ms/cycle
            double dtS = dt/1000;

            double[] error = new double[4];

            double avgError = 0;
            double avgPreviousError = 0;

            double avgArea = 0;
            double avgPreviousArea = 0;

            double Proportion = 0;

            double avgPower = 0;

            error[0] = tolerance + 1;
            error[1] = tolerance + 1;

            while (((Math.abs(error[0]) > tolerance) && (Math.abs(error[1]) > tolerance)) && opModeIsActive()) { // TODO: replace error[0] with avgError

                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newWheelTarget[0], newWheelTarget[1], newWheelTarget[2], newWheelTarget[3]);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", wheels[0].getCurrentPosition(), -wheels[1].getCurrentPosition(), -wheels[2].getCurrentPosition(), wheels[3].getCurrentPosition());

                telemetry.addData("Power: ", avgPower);

                telemetry.addData("Proportion:", Proportion);

                telemetry.addData("de(t)/dt", ((avgError - avgPreviousError) / dtS));

                telemetry.addData("error:", avgError);
                telemetry.addData("previous error:", avgPreviousError);

                telemetry.addData("Derivative:", kd * ((avgError - avgPreviousError) / dtS));
                telemetry.addData("Integral:", ki * avgArea);

                telemetry.addData("∫e(t)dt:", avgArea);
                telemetry.addData("previous ∫e(t)dt:", avgPreviousArea);

                telemetry.addData("dtS", dtS);

                telemetry.update();


                error[0] = (int)(distanceCM * COUNTS_PER_CM) - wheels[0].getCurrentPosition();
                error[1] = (int)(-distanceCM * COUNTS_PER_CM) - wheels[1].getCurrentPosition();
                error[2] = (int)(-distanceCM * COUNTS_PER_CM) - wheels[2].getCurrentPosition();
                error[3] = (int)(distanceCM * COUNTS_PER_CM) - wheels[3].getCurrentPosition();

                avgPreviousError = avgError;
                avgError = (Math.abs(error[0]) + Math.abs(error[1]) + Math.abs(error[2]) + Math.abs(error[3])) / 4;

                Proportion = Math.abs(avgError)/(int)(distanceCM * COUNTS_PER_CM);

                avgPreviousArea = avgArea;
                avgArea = avgArea * dtS + avgPreviousArea;

                avgPower = kp * Proportion + kd * ((avgArea - avgPreviousError) / dtS) + (ki * avgArea);

                wheels[0].setPower(avgPower);
                wheels[1].setPower(-avgPower);
                wheels[2].setPower(-avgPower);
                wheels[3].setPower(avgPower);

                sleep((long) dt);
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

    public void PIDStrafe(double distanceCM, double tolerance) { // TODO: Adjust Tolerance

        int []newWheelTarget = new int[4];

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            for (int i = 0; i < 4; i++) {
                newWheelTarget[i] = wheels[0].getCurrentPosition() + (int)(distanceCM * COUNTS_PER_CM); // TODO: Get avg position
            }

            double kp = 1;
            double kd = 0.00000008; // Constant of derivation
            double ki = 0.00006;

            double dt = 20; //Delta time = 20 ms/cycle
            double dtS = dt/1000;

            double[] error = new double[4];

            double avgError = 0;
            double avgPreviousError = 0;

            double avgArea = 0;
            double avgPreviousArea = 0;

            double Proportion = 0;

            double avgPower = 0;

            error[0] = tolerance + 1;
            error[1] = tolerance + 1;

            while (((Math.abs(error[0]) > tolerance) && (Math.abs(error[1]) > tolerance)) && opModeIsActive()) { // TODO: replace error[0] with avgError

                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newWheelTarget[0], -newWheelTarget[1], newWheelTarget[2], -newWheelTarget[3]);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", wheels[0].getCurrentPosition(), wheels[1].getCurrentPosition(), wheels[2].getCurrentPosition(), wheels[3].getCurrentPosition());

                telemetry.addData("Power: ", avgPower);

                telemetry.addData("Proportion:", Proportion);

                telemetry.addData("de(t)/dt", ((avgError - avgPreviousError) / dtS));

                telemetry.addData("error:", avgError);
                telemetry.addData("previous error:", avgPreviousError);

                telemetry.addData("Derivative:", kd * ((avgError - avgPreviousError) / dtS));
                telemetry.addData("Integral:", ki * avgArea);

                telemetry.addData("∫e(t)dt:", avgArea);
                telemetry.addData("previous ∫e(t)dt:", avgPreviousArea);

                telemetry.addData("dtS", dtS);

                telemetry.update();


                error[0] = (int)(distanceCM * COUNTS_PER_CM) - wheels[0].getCurrentPosition();
                error[1] = (int)(-distanceCM * COUNTS_PER_CM) - wheels[1].getCurrentPosition();
                error[2] = (int)(distanceCM * COUNTS_PER_CM) - wheels[2].getCurrentPosition();
                error[3] = (int)(-distanceCM * COUNTS_PER_CM) - wheels[3].getCurrentPosition();

                avgPreviousError = avgError;
                avgError = (Math.abs(error[0]) + Math.abs(error[1]) + Math.abs(error[2]) + Math.abs(error[3])) / 4;

                Proportion = Math.abs(avgError)/(int)(distanceCM * COUNTS_PER_CM);

                avgPreviousArea = avgArea;
                avgArea = avgArea * dtS + avgPreviousArea;

                avgPower = kp * Proportion + kd * ((avgArea - avgPreviousError) / dtS) + (ki * avgArea);

                wheels[0].setPower(avgPower);
                wheels[1].setPower(-avgPower);
                wheels[2].setPower(avgPower);
                wheels[3].setPower(-avgPower);

                sleep((long) dt);
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

    public void encoderCarouselSpinner(double speed, double spinnerMM, double timeOutS){
        int newCarouselSpinnerTarget;

        if (opModeIsActive()) {
            newCarouselSpinnerTarget = TIseBot.carouselSpinner.getCurrentPosition() + (int)(spinnerMM * COUNTS_PER_CM);

            TIseBot.carouselSpinner.setTargetPosition(newCarouselSpinnerTarget);

            TIseBot.carouselSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            TIseBot.carouselSpinner.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeOutS) && (TIseBot.carouselSpinner.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", newCarouselSpinnerTarget);
                telemetry.addData("Path2",  "Running at %7d", TIseBot.carouselSpinner.getCurrentPosition());
                telemetry.update();
            }
            TIseBot.carouselSpinner.setPower(0); // Resets encoders
            TIseBot.carouselSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TIseBot.carouselSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
