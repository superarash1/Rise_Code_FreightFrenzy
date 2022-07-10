package org.firstinspires.ftc.teamcode.AutonExamplesOld;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.otherCode.HardwareMap;

public class DistanceSensorExample extends LinearOpMode {

    HardwareMap robot = new HardwareMap();

    double fieldWidth = 96.0;
    double fieldDepth = 60.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        driveToTarget(67, 53, 1);

    }

    public double calculateAngle(){ //check if this is right
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return (angles.firstAngle - robot.straight.firstAngle); //Was kda working with +90 but was off by 20ish degrees
    }

    public double LocationCalculationX(){
        return robot.sensorLeft.getDistance(DistanceUnit.INCH) + 8;
    }

    public double LocationCalculationY(){
        return (fieldDepth - robot.sensorFront.getDistance(DistanceUnit.INCH)) - 5;
    }

    public void driveToTarget(double targetX, double targetY, double tolerance){
        double drive;
        double turn;
        double strafe;
        double fLeft;
        double fRight;
        double bLeft;
        double bRight;
        double max;

        double angle = tolerance + 1;

        boolean returnToPoint = false;
        boolean returnToPointA = false;

        double errorX = tolerance + 1;
        double errorY = tolerance + 1;

        while (opModeIsActive() && angle > tolerance){
            telemetry.addData("Angle", calculateAngle());

            angle = calculateAngle();

            // Adjust negatives to match the ones that make ur bot turn
            robot.frontLeftMotor.setPower(angle/180);
            robot.frontRightMotor.setPower(-angle/180);
            robot.backRightMotor.setPower(-angle/180);
            robot.backLeftMotor.setPower(angle/180);
        }

        while (opModeIsActive() && (errorX > tolerance || errorY > tolerance)){
            telemetry.addData("X-Location", LocationCalculationX());
            telemetry.addData("Y-Location", LocationCalculationY());

            strafe = (LocationCalculationX() - targetX) / targetX;
            drive = (LocationCalculationY() - targetY) / targetY;

            // Adjust negatives to match the direction
            fLeft = -.7*drive - .8*strafe;
            fRight = -.7*drive + .8*strafe;
            bRight = -.7*drive - .8*strafe;
            bLeft = -.7*drive + .8*strafe;

            max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
            if (max > 1.0) {
                fLeft /= max;
                fRight /= max;
                bLeft /= max;
                bRight /= max;
            }

            robot.frontRightMotor.setPower(fRight);
            robot.frontLeftMotor.setPower(fLeft);
            robot.backRightMotor.setPower(bRight);
            robot.backLeftMotor.setPower(bLeft);
        }

        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);

        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
