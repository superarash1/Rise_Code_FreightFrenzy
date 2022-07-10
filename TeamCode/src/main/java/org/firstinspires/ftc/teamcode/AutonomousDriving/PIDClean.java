package org.firstinspires.ftc.teamcode.AutonomousDriving;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.OldFFCode.Hardware;

public class PIDClean extends LinearOpMode {
    Hardware TIseBot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    // Constants to find the amount of encoder ticks per CM
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_CM = 9.60;

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
        PID("drive",10,3);
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

    public void PID(String type, double distance, double tolerance){
        int []newWheelTarget = new int[4];
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            for (int i = 0; i < 4; i++) {
                newWheelTarget[i] = wheels[0].getCurrentPosition() + (int)(distance * COUNTS_PER_CM); // TODO: Get avg position
            }

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

            switch (type) {
                case "drive":
                    kp = 1;
                    kd = 0.00018; // Constant of derivation
                    ki = 0.000009;

                    kV = 0;
                    kStatic = 0;

                    referenceVelocity = 0;

                    dt = 50; //Delta time = 20 ms/cycle
                    dtS = dt/1000;

                    error = tolerance + 1;

                    while ((Math.abs(error) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError

                        telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newWheelTarget[0], newWheelTarget[1], newWheelTarget[2], newWheelTarget[3]);
                        telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", wheels[0].getCurrentPosition(), wheels[1].getCurrentPosition(), wheels[2].getCurrentPosition(), wheels[3].getCurrentPosition());

                        telemetry.addData("DistanceCM: ", (int)(distance * COUNTS_PER_CM));

                        telemetry.addData("power: ", power);

                        telemetry.addData("Friction FeedForward:", kStatic * Math.signum(PID));
                        telemetry.addData("Velocity FeedForward:", kV * (referenceVelocity * Math.signum(PID)));

                        telemetry.addData("Proportion:", P);
                        telemetry.addData("Derivative:", kd * ((error - previousError) / dtS));
                        telemetry.addData("Integral:", ki * area);

                        telemetry.addData("de(t)/dt", ((error - previousError) / dtS));

                        telemetry.addData("error:", error);
                        telemetry.addData("previous error:", previousError);

                        telemetry.addData("∫e(t)dt:", area);
                        telemetry.addData("previous ∫e(t)dt:", previousArea);

                        telemetry.addData("dtS", dtS);

                        telemetry.update();

                        previousError = error;

                        error = (int)(distance * COUNTS_PER_CM) - wheels[0].getCurrentPosition();

                        P = Math.abs(error)/(int)(distance * COUNTS_PER_CM);

                        previousArea = area;

                        if (error*(int)(distance * COUNTS_PER_CM) < 0) previousArea = 0;

                        area = error * dtS + previousArea;

                        PID = kp * P + kd * ((error - previousError) / dtS) + (ki * area);

                        power = PID + kStatic * Math.signum(PID) + kV * (referenceVelocity * Math.signum(PID));

                        wheels[0].setPower(power);
                        wheels[1].setPower(power);
                        wheels[2].setPower(power);
                        wheels[3].setPower(power);

                        sleep((long) dt);
                    }
                    break;

                case "turn":
                    double totalDistance = distance - calculateTotalAngle();

                    kp = 1;
                    kd = 0.0009;
                    ki = 0.00075;

                    kV = 0;
                    kStatic = 0;

                    referenceVelocity = 0;

                    dt = 20;
                    dtS = dt/1000;

                    error = tolerance + 1;

                    while (opModeIsActive() && (Math.abs(error) > tolerance)) {

                        telemetry.addData("Target Angle: ", distance);
                        telemetry.addData("Current Angle: ", calculateTotalAngle());

                        telemetry.addData("Total Distance", distance);

                        telemetry.addData("Power", power);

                        telemetry.addData("de(t)/dt: ", (error - previousError)/dtS);

                        telemetry.addData("Proportion: ", P);

                        telemetry.addData("Error: ", error);

                        telemetry.addData("Derivative: ", kd * ((error - previousError)/dtS));
                        telemetry.addData("Integral: ", ki * area);


                        telemetry.addData("Previous Error: ", previousError);

                        telemetry.addData("Area: ", area);
                        telemetry.addData("Previous Area: ", previousArea);

                        telemetry.addData("dtS", dtS);

                        telemetry.update();

                        previousError = error;

                        error = distance - calculateTotalAngle();

                        P = Math.abs(error)/totalDistance;


                        previousArea = area;
                        area = (error * dtS) + previousArea;
                        power = kp * P + kd * ((error - previousError)/dtS) + ki * area;


                        wheels[0].setPower(power);
                        wheels[1].setPower(-power);
                        wheels[2].setPower(-(power)*1.5);
                        wheels[3].setPower((power)*1.5);

                        sleep((long) dt);
                    }
                    break;

                case "strafe":

                    kp = 1;
                    kd = 0.0002; // Constant of derivation
                    ki = 0.00008;

                    referenceVelocity = 0;

                    dt = 40; //Delta time = 20 ms/cycle
                    dtS = dt/1000;

                    kV = 0;
                    kStatic = 0;

                    error = tolerance + 1;

                    while ((Math.abs(error) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError

                        telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newWheelTarget[0], newWheelTarget[1], newWheelTarget[2], newWheelTarget[3]);
                        telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", wheels[0].getCurrentPosition(), -wheels[1].getCurrentPosition(), -wheels[2].getCurrentPosition(), wheels[3].getCurrentPosition());

                        telemetry.addData("Power: ", power);

                        telemetry.addData("Proportion:", P);

                        telemetry.addData("Derivative:", kd * ((error - previousError) / dtS));
                        telemetry.addData("Integral:", ki * area);

                        telemetry.addData("error:", error);
                        telemetry.addData("previous error:", previousError);

                        telemetry.addData("de(t)/dt", ((error - previousError) / dtS));

                        telemetry.addData("∫e(t)dt:", area);
                        telemetry.addData("previous ∫e(t)dt:", previousArea);

                        telemetry.addData("dtS", dtS);

                        telemetry.update();

                        previousError = error;
                        error = (int)(distance * COUNTS_PER_CM) - wheels[0].getCurrentPosition();

                        P = Math.abs(error)/(int)(distance * COUNTS_PER_CM);

                        previousArea = area;
                        area = error * dtS + previousArea;

                        PID = kp * P + kd * ((error - previousError) / dtS) + (ki * area);

                        power = PID + kStatic * Math.signum(PID) + kV * (referenceVelocity * Math.signum(PID));

                        wheels[0].setPower(power);
                        wheels[1].setPower(-power);
                        wheels[2].setPower(-power);
                        wheels[3].setPower(power);

                        sleep((long) dt);
                    }
                    break;
                default:
                    telemetry.addLine("Invalid Input");
                    telemetry.update();
                    break;
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
}
