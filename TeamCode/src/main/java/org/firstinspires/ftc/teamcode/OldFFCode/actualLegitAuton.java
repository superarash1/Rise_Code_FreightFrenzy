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
import org.firstinspires.ftc.teamcode.OldFFCode.Hardware;

@Disabled
public class actualLegitAuton extends LinearOpMode {
    Hardware TIseBot = new Hardware();

    private ElapsedTime runtime = new ElapsedTime();
    // Constants to find the amount of encoder ticks per CM
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_CM = 9.60;

    static final double ARM_COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV / 360) * 0.48;

    public double realAngle;

    // Finds the amount of encoder ticks per CM
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    DcMotor[] wheels = new DcMotor[4];

    Orientation lastAngles = new Orientation();
    public double globalAngle;
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

        PIDDrive(50, 3);
        sleep(50);
        armEncoder(100,20);
        sleep(500);
        PIDDrive(-42, 2);
        sleep(50);
        IMU_Turn_PID_Total2(0,3);
        sleep(50);
        PIDStrafe(-123,4);
        sleep(50);
        carouselSpinner(-3000, 200);
        sleep(50);

        PIDDrive(7, 3);
        sleep(50);

        IMU_Turn_PID_Total(90,3);
        sleep(50);

        encoderDrive(1, 280);

        armEncoder(-90,15);


//        encoderDrive(0.07, 11);
//        sleep(50);
//

//
//        PIDDrive(30, 3);
//        sleep(50);
//




//        TIseBot.gate.setPosition(0.2);
//        sleep(500);
//
//        IMU_Turn_PID_Total(90,2);



    }

    public double calculateTotalAngle(){
        Orientation angles = TIseBot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = angles.firstAngle - TIseBot.straight.firstAngle;

        return -globalAngle;
    }

    private void resetAngle() {
        lastAngles = TIseBot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double calculateRelativeAngle() {
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

        return -globalAngle;
    }

    public void carouselSpinner (int distance, double tolerance){

//        double error = tolerance + 1;

        TIseBot.cSpinRight.setTargetPosition(distance);
        TIseBot.cSpinRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        TIseBot.cSpinRight.setPower(0.5);

        while (TIseBot.cSpinRight.isBusy() && opModeIsActive()){

            telemetry.addData("Position", TIseBot.cSpinRight.getCurrentPosition());
            telemetry.update();
//            error = Math.abs(distance) - Math.abs(TIseBot.cSpinRight.getCurrentPosition());

        }
        TIseBot.cSpinRight.setPower(0);
        TIseBot.cSpinRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TIseBot.cSpinRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void armEncoder(double degree, double tolerance){
        double error = tolerance + 1;
        double P = 0;

        double kp = 0.1;


        while (Math.abs(error) > tolerance && opModeIsActive()){
            realAngle = TIseBot.arm1.getCurrentPosition() * ARM_COUNTS_PER_DEGREE;
            telemetry.addData("error", error);
            telemetry.addData("target:", degree*ARM_COUNTS_PER_DEGREE);
            telemetry.addData("power", P);
            telemetry.update();
            error = (degree*ARM_COUNTS_PER_DEGREE) - TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE;

            P = error/Math.abs(degree);

            TIseBot.arm1.setPower(kp*P);
            TIseBot.arm2.setPower(-kp*P);
        }

        TIseBot.arm1.setPower(0);
        TIseBot.arm2.setPower(0);
    }

    public void encoderDrive(double speed, double distance) {

        // Creates an array for the target positions for each of your motors
        int []newWheelTarget = new int[4];

        if (opModeIsActive()) {

            // Defines the target position for your motor
            newWheelTarget[0] = wheels[0].getCurrentPosition() + (int)(distance * COUNTS_PER_CM);
            newWheelTarget[1] = wheels[1].getCurrentPosition() + (int)(distance * COUNTS_PER_CM);
            newWheelTarget[2] = wheels[2].getCurrentPosition() + (int)(distance * COUNTS_PER_CM);
            newWheelTarget[3] = wheels[3].getCurrentPosition() + (int)(distance * COUNTS_PER_CM);

            for (int i = 0; i < 4; i++) {
                // Sets the target position for the motors
                wheels[i].setTargetPosition(newWheelTarget[i]);

                // Tells the motor to drive until they reach the target position
                wheels[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            // reset the timeout time and start motion.

            // Sets speed (negation indicates direction;
            // in this case negative is forwards because that's how the motor orientation was set up in the Hardware map and I don't wanna change all of my code to make forward positive)
            TIseBot.frontLeftMotor.setPower(-speed);
            TIseBot.frontRightMotor.setPower(-speed);
            TIseBot.backRightMotor.setPower(-speed);
            TIseBot.backLeftMotor.setPower(-speed);

            // Will keep looping until one of the of the motors reach the target position
            while (opModeIsActive() && (TIseBot.frontLeftMotor.isBusy() && TIseBot.frontRightMotor.isBusy() && TIseBot.backRightMotor.isBusy() && TIseBot.backLeftMotor.isBusy())) {
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
            double kd = 0; // Constant of derivation  0.00018
            double ki = 0; //0.0000009

            double kStatic = 0;
            double kV = 0;

            double referenceVelocity = 0;

            double dt = 50; //Delta time = 20 ms/cycle
            double dtS = dt/1000;

            double[] error = new double[4];
            double[] previousError = new double[4];

            double[] area = new double[4];
            double[] previousArea = new double[4];

            double[] PID = new double[4];
            double[] power = new double[4];


            error[2] = tolerance + 1;

            while ((Math.abs(error[2]) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError

                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newWheelTarget[0], newWheelTarget[1], newWheelTarget[2], newWheelTarget[3]);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", wheels[0].getCurrentPosition(), wheels[1].getCurrentPosition(), wheels[2].getCurrentPosition(), wheels[3].getCurrentPosition());

                telemetry.addData("DistanceCM: ", (int)(distanceCM * COUNTS_PER_CM));

                telemetry.addData("power: ", power[2]);

                telemetry.addData("Friction FeedForward:", kStatic * Math.signum(PID[2]));
                telemetry.addData("Velocity FeedForward:", kV * (referenceVelocity * Math.signum(PID[2])));

                telemetry.addData("Proportion:", P[2]);
                telemetry.addData("Derivative:", kd * ((error[2] - previousError[2]) / dtS));
                telemetry.addData("Integral:", ki * area[2]);

                telemetry.addData("de(t)/dt", ((error[2] - previousError[2]) / dtS));

                telemetry.addData("error:", error[2]);
                telemetry.addData("previous error:", previousError[2]);

                telemetry.addData("∫e(t)dt:", area[2]);
                telemetry.addData("previous ∫e(t)dt:", previousArea[2]);

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
                P[2] = (error[2])/Math.abs((int)(distanceCM * COUNTS_PER_CM));
                P[3] = Math.abs(error[3])/(int)(distanceCM * COUNTS_PER_CM);

                previousArea[0] = area[0];
                previousArea[1] = area[1];
                previousArea[2] = area[2];
                previousArea[3] = area[3];

                if (error[0]*(int)(distanceCM * COUNTS_PER_CM) < 0) previousArea[0] = 0;
                if (error[1]*(int)(distanceCM * COUNTS_PER_CM) < 0) previousArea[1] = 0;
                if (error[2]*(int)(distanceCM * COUNTS_PER_CM) < 0) previousArea[2] = 0;
                if (error[3]*(int)(distanceCM * COUNTS_PER_CM) < 0) previousArea[3] = 0;

                area[0] = error[0] * dtS + previousArea[0];
                area[1] = error[1] * dtS + previousArea[1];
                area[2] = error[2] * dtS + previousArea[2];
                area[3] = error[3] * dtS + previousArea[3];

                PID[0] = kp * P[0] + kd * ((error[0] - previousError[0]) / dtS) + (ki * area[0]);
                PID[1] = kp * P[1] + kd * ((error[1] - previousError[1]) / dtS) + (ki * area[1]);
                PID[2] = kp * P[2] + kd * ((error[2] - previousError[2]) / dtS) + (ki * area[2]);
                PID[3] = kp * P[3] + kd * ((error[3] - previousError[3]) / dtS) + (ki * area[3]);

                power[0] = PID[0] + kStatic * Math.signum(PID[0]) + kV * (referenceVelocity * Math.signum(PID[0]));
                power[1] = PID[1] + kStatic * Math.signum(PID[1]) + kV * (referenceVelocity * Math.signum(PID[1]));
                power[2] = PID[2] + kStatic * Math.signum(PID[2]) + kV * (referenceVelocity * Math.signum(PID[2]));
                power[3] = PID[3] + kStatic * Math.signum(PID[3]) + kV * (referenceVelocity * Math.signum(PID[3]));

                wheels[0].setPower(PID[2]);
                wheels[1].setPower(PID[2]);
                wheels[2].setPower((PID[2]));
                wheels[3].setPower((PID[2 ]));

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

    public void IMU_Turn_PID_Total(double degree, double tolerance) {

        double totalDistance = degree - calculateTotalAngle();

        double kp = 1;
        double kd = 0; //0.0009
        double ki = 0; //0.00075;

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


            wheels[0].setPower(power);
            wheels[1].setPower(-power);
            wheels[2].setPower(-(power)*1.5);
            wheels[3].setPower((power)*1.5);

            sleep((long) dt);
        }

        for (int i = 0; i < 4; i++){
            wheels[i].setPower(0);
        }
    }

    public void IMU_Turn_PID_Total2(double degree, double tolerance) {

        double totalDistance = degree - calculateTotalAngle();

        double kp = 0.05;
        double kd = 0; //0.0009
        double ki = 0; //0.00075;

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


            wheels[0].setPower(power);
            wheels[1].setPower(-power);
            wheels[2].setPower(-(power)*1.5);
            wheels[3].setPower((power)*1.5);

            sleep((long) dt);
        }

        for (int i = 0; i < 4; i++){
            wheels[i].setPower(0);
        }
    }

    public void PIDStrafe(double distanceCM, double tolerance)  { // TODO: Adjust Tolerance

        int []newWheelTarget = new int[4];

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            for (int i = 0; i < 4; i++) {
                newWheelTarget[i] = wheels[0].getCurrentPosition() + (int)(distanceCM * COUNTS_PER_CM); // TODO: Get avg position
            }

            double kp = 0.8;
            double kd = 0; // Constant of derivation  0.0002
            double ki = 0; // 0.00008

            double dt = 40; //Delta time = 20 ms/cycle
            double dtS = dt/1000;

            double[] error = new double[4];

            double avgError = 0;
            double avgPreviousError = 0;

            double avgArea = 0;
            double previousArea = 0;

            double Derivative = 0;

            double Proportion = 0;

            double avgPower = 0;

            error[1] = tolerance + 1;

            while ((Math.abs(error[1]) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError


                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newWheelTarget[0], newWheelTarget[1], newWheelTarget[2], newWheelTarget[3]);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", wheels[0].getCurrentPosition(), -wheels[1].getCurrentPosition(), -wheels[2].getCurrentPosition(), wheels[3].getCurrentPosition());

                telemetry.addData("Power: ", avgPower);

                // telemetry.addData("previousArea * avgArea: ", previousArea * avgArea);

                telemetry.addData("Proportion:", Proportion);

                telemetry.addData("de(t)/dt", Derivative);

                telemetry.addData("Derivative:", kd * ((error[1] - avgPreviousError) / dtS));
                //telemetry.addData("Integral:", ki * avgArea);

                telemetry.addData("error:", error[1]);
                telemetry.addData("previous error:", avgPreviousError);

                telemetry.addData("de(t)/dt", ((error[1] - avgPreviousError) / dtS));

                telemetry.addData("∫e(t)dt:", avgArea);
                telemetry.addData("previous ∫e(t)dt:", previousArea);

                telemetry.addData("dtS", dtS);

                telemetry.update();

                avgPreviousError = error[1];

                error[0] = (int)(distanceCM * COUNTS_PER_CM) - wheels[0].getCurrentPosition();
                error[1] = (int)(-distanceCM * COUNTS_PER_CM) - wheels[1].getCurrentPosition();
                error[2] = (int)(-distanceCM * COUNTS_PER_CM) - wheels[2].getCurrentPosition();
                error[3] = (int)(distanceCM * COUNTS_PER_CM) - wheels[3].getCurrentPosition();

                Proportion = error[1]/Math.abs((int)(distanceCM * COUNTS_PER_CM));

                previousArea = avgArea;
                avgArea = error[1] * dtS + previousArea;

                if ((error[1] * distanceCM) < 0){
                    previousArea = 0;
                }

                avgPower = kp * Proportion + kd * Derivative + (ki * avgArea);

                wheels[0].setPower(-avgPower);
                wheels[1].setPower(avgPower);
                wheels[2].setPower(-(avgPower));
                wheels[3].setPower((avgPower));

                Derivative = (error[1] - avgPreviousError) / dtS;

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

    public void distanceDrive(double x, double y){
        
    }
}
