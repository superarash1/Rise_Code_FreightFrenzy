package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class TIseBotTeleOp extends LinearOpMode {

    /* Declare OpMode members */
    Hardware TIseBot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    public double globalAngle;
    Orientation lastAngles = new Orientation();

    public double spinnerCM = 2;

    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double ARM_COUNTS_PER_DEGREE = COUNTS_PER_MOTOR_REV / 360;

    DcMotor[] wheels = new DcMotor[4];

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

        double coolDown = 25;

        // Initialize the hardware variables using the init() method in the hardware class
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


        TIseBot.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TIseBot.arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TIseBot.arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Press Button to Activate OpMode");
        telemetry.update();

        waitForStart();

        Double_Arm_PID(spinnerCM, 1);
        TIseBot.arm1.setTargetPosition((int) (((int) spinnerCM + (TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE))/10));
        TIseBot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        TIseBot.arm2.setTargetPosition((int) (((int) 75 + (TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE))/10));
        TIseBot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        coolDown = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){

            // Gamepad controls for movement
            drive = gamepad1.left_stick_y; //Between -1 and 1
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            // Mechanum Drive Calculations
            fLeft = -.7*drive + .8*strafe + .75*turn;
            fRight = -.7*drive - .8*strafe - .75*turn;
            bRight = -.7*drive + .8*strafe - .75*turn;
            bLeft = -.7*drive - .8*strafe + .75*turn;

            // This ensures that the power values the motors are set to are in the range (-1, 1)
            max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
            if (max > 1.0) {
                fLeft /= max;
                fRight /= max;
                bLeft /= max;
                bRight /= max;
            }

            // Set power to the motors
            TIseBot.frontRightMotor.setPower((fRight));
            TIseBot.frontLeftMotor.setPower(fLeft);
            TIseBot.backRightMotor.setPower((bRight)*1.5);
            TIseBot.backLeftMotor.setPower((bLeft)*1.5);

            if(gamepad1.right_trigger > 0){
                TIseBot.intake.setPower(gamepad1.right_trigger*0.2);
            } else if(gamepad1.left_trigger > 0){
                TIseBot.intake.setPower(-(gamepad1.left_trigger*0.2));
            } else{
                TIseBot.intake.setPower(0);
            }

            coolDown++;

            if (gamepad1.dpad_up && coolDown > 20){
                spinnerCM += 10;
                Double_Arm_PID(spinnerCM,1);
                TIseBot.arm1.setTargetPosition((int) (((int) spinnerCM + (TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE))/10));
                TIseBot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                TIseBot.arm2.setTargetPosition((int) (((int) 75 + (TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE))/10));
                TIseBot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                coolDown = 0;
            }

            if (gamepad1.dpad_down && coolDown > 20){
                spinnerCM -= 10;
                Double_Arm_PID(spinnerCM,1);
                TIseBot.arm1.setTargetPosition((int) (((int) spinnerCM + (TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE))/10));
                TIseBot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                TIseBot.arm2.setTargetPosition((int) (((int) spinnerCM + (TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE))/10));
                TIseBot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                coolDown = 0;
            }

            if (gamepad1.dpad_right){
                TIseBot.cageSpin1.setPosition(TIseBot.cageSpin1.getPosition() + 2);
                TIseBot.cageSpin2.setPosition(TIseBot.cageSpin2.getPosition() + 2);
            }

            if (gamepad1.dpad_left){
                TIseBot.cageSpin1.setPosition(TIseBot.cageSpin1.getPosition() - 2);
                TIseBot.cageSpin2.setPosition(TIseBot.cageSpin2.getPosition() - 2);
            }

            if (gamepad1.a){
                TIseBot.gate.setPosition(TIseBot.cageSpin1.getPosition() + 100);
            }

            if (gamepad1.b){
                TIseBot.gate.setPosition(TIseBot.cageSpin1.getPosition() - 100);
            }

            // Pause for 20 mS each cycle = update 50 times a second.
            sleep(20);
        }

        TIseBot.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    // Finds the current angle of the robot relative to its initial position
    public double calculateAngle(){ // TODO: Check axes order in teleOp and test with radians for fun
        Orientation angles = TIseBot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle - TIseBot.straight.firstAngle;
    }

    private void resetAngle() {
        lastAngles = TIseBot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
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

        return globalAngle;
    }

    // TODO: Code Carousel Spinner
    public void carouselSpinner(){

    }

    public void Double_Arm_PID(double Degrees, double tolerance) { // TODO: convert spinnerCM to degrees

        double kp = 1;
        double kd = 0.000;
        double ki = 0.000;

        // TODO: kGravity

        double kGravity = 0;
        double kV = 0;

        double referenceVelocity = 0;

        double PID = 0;
        double power = 0;

        double P = 0;

        double previousError = 0;
        double error = tolerance + 1;

        double area = 0;
        double previousArea = 0;

        double dt = 40;
        double dts = dt/1000;

        while (opModeIsActive()) {

            telemetry.addData("Target Angle: ", TIseBot.arm1.getTargetPosition());
            telemetry.addData("Current Angle: ", TIseBot.arm1.getCurrentPosition());

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
            error = (TIseBot.arm1.getTargetPosition() - TIseBot.arm1.getCurrentPosition());

            previousArea = area;
            area = (error * dts) + previousArea;

            P = Math.abs(error)/(Degrees);

            PID = kp * P + kd * ((error - previousError)/dts) + ki * area;
            power = PID + kGravity * Math.sin(TIseBot.arm1.getTargetPosition()) + kV * (referenceVelocity * Math.signum(PID)); // TODO: potentially change to sine

            TIseBot.arm1.setPower(-power);
            TIseBot.arm2.setPower(power);

            sleep((long) dt);
        }

        TIseBot.arm1.setPower(0);
        TIseBot.arm2.setPower(0);

        // Resets encoders
        TIseBot.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TIseBot.arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TIseBot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TIseBot.arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
