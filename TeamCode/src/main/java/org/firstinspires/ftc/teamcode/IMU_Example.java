package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
public class IMU_Example extends LinearOpMode {
    Hardware TIseBot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

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
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        IMU_Turn_Relative(-100, 0.5,0.5);
        sleep(250);

        IMU_Turn_Total(0, 0.5, 0.5);
        sleep(250);

        IMU_Turn_PID_Relative(-100, 0.5);
        sleep(250);

        IMU_Turn_PID_Total(0, 0.5);
        sleep(250);
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

    public void IMU_Turn_Total(double degree, double speed, double tolerance) {

        double error = tolerance + 1;
        while (opModeIsActive() && (error > tolerance)) {

            telemetry.addData("Target Angle: ", degree);
            telemetry.addData("Current Angle: ", calculateTotalAngle());
            telemetry.update();

            error = Math.abs(degree - calculateTotalAngle());

            if (calculateTotalAngle() > degree + tolerance) {
                wheels[0].setPower(speed);
                wheels[1].setPower(-speed);
                wheels[2].setPower(-speed);
                wheels[3].setPower(speed);
            }

            if (calculateTotalAngle() < degree - tolerance) {
                wheels[0].setPower(-speed);
                wheels[1].setPower(speed);
                wheels[2].setPower(speed);
                wheels[3].setPower(-speed);
            }

            sleep(20);
        }

        for (int i = 0; i < 4; i++){
            wheels[i].setPower(0);
        }
    }

    public void IMU_Turn_Relative(double degree, double speed, double tolerance) {

        double error = tolerance + 1;
        while (opModeIsActive() && (Math.abs(error) > tolerance)) {

            telemetry.addData("Target Angle: ", degree);
            telemetry.addData("Current Angle: ", calculateRelativeAngle());
            telemetry.update();

            error = Math.abs(degree - calculateTotalAngle());

            if (calculateRelativeAngle() > degree + tolerance) {
                wheels[0].setPower(speed);
                wheels[1].setPower(-speed);
                wheels[2].setPower(-speed);
                wheels[3].setPower(speed);
            }

            if (calculateRelativeAngle() < degree - tolerance) {
                wheels[0].setPower(-speed);
                wheels[1].setPower(speed);
                wheels[2].setPower(speed);
                wheels[3].setPower(-speed);
            }

            sleep(20);
        }

        for (int i = 0; i < 4; i++){
            wheels[i].setPower(0);
            resetAngle();
        }
    }

    public void IMU_Turn_PID_Total(double degree, double tolerance) {

        double totalDistance = degree - calculateTotalAngle();

        double kp = 1;
        double kd = 0.0009;
        double ki = 0.00075;

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

    public void IMU_Turn_PID_Relative(double degree, double tolerance) {

        double kp = 1;
        double kd = 0.0009;
        double ki = 0.00075;

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
            telemetry.addData("Current Angle: ", calculateRelativeAngle());

            telemetry.addData("Power", power);

            telemetry.addData("de(t)/dt: ", (error - previousError)/dts);

            telemetry.addData("Proportion: ", P);
            telemetry.addData("Derivative: ", kd * ((error - previousError)/dts));
            telemetry.addData("Integral: ", ki * area);

            telemetry.addData("Error: ", error);
            telemetry.addData("Previous Error: ", previousError);

            telemetry.addData("Area: ", area);
            telemetry.addData("Previous Area: ", previousArea);

            telemetry.addData("dtS", dts);

            telemetry.update();

            previousError = error;

            error = degree - calculateRelativeAngle();

            previousArea = area;
            area = (error * dts) + previousArea;

            P = Math.abs(error)/degree;

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
