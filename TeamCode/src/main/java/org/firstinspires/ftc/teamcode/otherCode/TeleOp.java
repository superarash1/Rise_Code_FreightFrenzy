package org.firstinspires.ftc.teamcode.otherCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
public class TeleOp extends LinearOpMode {

    HardwareMap robot = new HardwareMap();
    private ElapsedTime runtime = new ElapsedTime();

    //random size for field I will build at home
    double fieldWidth = 96.0;
    double fieldDepth = 60.0;
    double pointLocationX = 45.0;
    double pointLocationY = 25.0;
    double degreeOfFreedom = 1;

    public double angle;

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

        boolean returnToPoint = false;
        boolean returnToPointA = false;


        robot.init(hardwareMap);

        robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("X-Location", LocationCalculationX());
            telemetry.addData("Y-Location", LocationCalculationY());
            telemetry.addData("Angle", calculateAngle());

            drive = gamepad1.left_stick_y; //Between -1 and 1
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.dpad_left? -1:gamepad1.dpad_right? 1: gamepad1.left_stick_x;

            //when told to return to location, this will adjust the angle
            if (gamepad1.x){
                returnToPoint = true;
            }

            if (returnToPoint){ //check if this is right
                angle = calculateAngle(); // reupload decent code cuz I forgot to before
                if (Math.abs(angle) < 2){ //switch to Math.abs(angle) < ___ if absolute values of the numbers are the same
                    turn = 0;

                    returnToPointA = true;

                } else{
                    turn = angle/180; //make slower increase accuracy???
                }
            }

            if (returnToPointA) {
                returnToPoint = false;

                strafe = -((LocationCalculationX() - pointLocationX) / pointLocationX);
                drive = ((LocationCalculationY() - pointLocationY) / pointLocationY);
                
                if (LocationCalculationX() > (pointLocationX - degreeOfFreedom) && LocationCalculationX() < (pointLocationX + degreeOfFreedom)) {
                    strafe = gamepad1.dpad_left ? -1 : gamepad1.dpad_right ? 1 : gamepad1.left_stick_x;
                }
                if (LocationCalculationY() > (pointLocationY - degreeOfFreedom) && LocationCalculationY() < (pointLocationY + degreeOfFreedom)) {
                    drive = gamepad1.left_stick_y; //Between -1 and 1
                }
                if ((LocationCalculationX() > (pointLocationX - degreeOfFreedom) && LocationCalculationX() < (pointLocationX + degreeOfFreedom)) && (LocationCalculationY() > (pointLocationY - degreeOfFreedom) && LocationCalculationY() < (pointLocationY + degreeOfFreedom))){
                    if (Math.abs(angle) < 0.5){
                        turn = 0;
                    } else{
                        turn = angle/180;
                    }
                    returnToPointA = false;
                }
            }

            // Mechanum Drive Calculations
            fLeft = -.7*drive + .8*strafe - .75*turn;
            fRight = -.7*drive - .8*strafe + .75*turn;
            bRight = -.7*drive + .8*strafe + .75*turn;
            bLeft = -.7*drive - .8*strafe - .75*turn;

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

            telemetry.update();
            sleep(20);
        }
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
}
