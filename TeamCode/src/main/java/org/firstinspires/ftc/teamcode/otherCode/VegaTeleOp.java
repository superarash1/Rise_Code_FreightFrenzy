package org.firstinspires.ftc.teamcode.otherCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
public class VegaTeleOp extends LinearOpMode {
    VegaHardware robot = new VegaHardware();

    private ElapsedTime runtime = new ElapsedTime();
    public double angle;

    double ARM_COUNTS_PER_DEGREE = (537.7/360);

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

        boolean toggle1 = true;
        boolean toggle2 = false;
        //double armPosition = robot.ARM_HOME;
        //final double ARM_SPEED = 0.01;
        double armMovement;

        boolean sliderToggle = false;
        boolean armToggle = false;
        boolean sliderStay = false;
        double sliderStayTime = 0;
        double armStartTime = 0;
        double startTime = 0;

        robot.init(hardwareMap);

       // robot.frontRightMotor.set

//    .roPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()){

//            drive = gamepad1.left_stick_y; //Between -1 and 1
//            turn = gamepad1.right_stick_x;
//            strafe = gamepad1.left_stick_x;

            double seconds = runtime.seconds();

            drive = Math.pow(gamepad1.left_stick_y, 3); //Between -1 and 1
            turn = Math.pow(gamepad1.right_stick_x, 3);
            strafe = Math.pow(gamepad1.left_stick_x, 3);
            //servo calculations

            // Mecanum Drive Calculations
            fLeft = -.7*drive + .7*strafe + .7*turn;
            fRight = -.7*drive - .7*strafe - .7*turn;
            bRight = -.7*drive + .7*strafe - .7*turn;
            bLeft = -.7*drive - .7*strafe + .7*turn;

            max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
            if (max > 1.0) {
                fLeft /= max;
                fRight /= max;
                bLeft /= max;
                bRight /= max;
            }

            robot.frontRight.setPower(fRight);
            robot.frontLeft.setPower(fLeft);
            robot.backRight.setPower(bRight);
            robot.backLeft.setPower(bLeft);


            if (gamepad1.right_bumper){
                robot.arm.setPower(.35  );

            } else if (gamepad1.left_bumper){
                robot.arm.setPower(-.35);
            } else {
                robot.arm.setPower(0);
            }

            robot.slide.setPower(-1*(gamepad1.left_trigger));

            robot.slide.setPower(1*(gamepad1.right_trigger));

            if (gamepad1.dpad_down) {
                robot.intake.setPower(1);
            }

            if (gamepad1.right_stick_button) {
                robot.intake.setPower(1);
            }
            else if (gamepad1.left_stick_button) {
                robot.intake.setPower(-1);
            }  else {
                robot.intake.setPower(0);
            }

            if (gamepad1.dpad_left)  {
                robot.spinner.setPower(1);
            } else {
                robot.spinner.setPower(0);
            }

            if (gamepad1.dpad_right) {
                robot.spinner2.setPower(1);
            } else  {
                robot.spinner2.setPower(0);
            }

            telemetry.update();
            sleep(20);
        }
    }

    public double calculateAngle(){ //check if this is right
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return (angles.firstAngle - robot.straight.firstAngle); //Was kda working with +90 but was off by 20ish degrees
    }


    public void autoScore(){
        robot.slide.setTargetPosition(1080);
        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.setPower(0.6);

        if (robot.slide.getCurrentPosition()/ARM_COUNTS_PER_DEGREE >= 1000){
            robot.arm.setTargetPosition((int)(200*ARM_COUNTS_PER_DEGREE));
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(0.3);
            while (robot.arm.isBusy()){
                telemetry.addData("Arm Position:", robot.arm.getCurrentPosition()/ARM_COUNTS_PER_DEGREE);
                telemetry.addData("Target Position:", robot.arm.getTargetPosition()/ARM_COUNTS_PER_DEGREE);
            }
            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.arm.setPower(0);
        }

        robot.slide.setPower(0);
        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (robot.slide.getCurrentPosition()/ARM_COUNTS_PER_DEGREE < 20){
            robot.arm.setTargetPosition((int)(-200*ARM_COUNTS_PER_DEGREE));
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(-0.3);
            while (robot.arm.isBusy()){
                telemetry.addData("Arm Position:", robot.arm.getCurrentPosition()/ARM_COUNTS_PER_DEGREE);
                telemetry.addData("Target Position:", robot.arm.getTargetPosition()/ARM_COUNTS_PER_DEGREE);
            }
            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.arm.setPower(0);
        }
    }


}
