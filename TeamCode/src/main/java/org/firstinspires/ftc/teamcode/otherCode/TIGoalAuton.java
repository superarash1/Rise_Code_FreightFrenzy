package org.firstinspires.ftc.teamcode.otherCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.*;


//@Autonomous(name="TIGoalAuton", group="TIBot")
@Disabled
public class TIGoalAuton extends LinearOpMode {

    /* Declare OpMode members. */
    TIGoalHardware robot = new TIGoalHardware();
    private ElapsedTime runtime = new ElapsedTime();

    final double encoderTicksPerRev = 537.6;
    final double wheelRadius = 2;
    final double wheelCircumference = 2*Math.PI*wheelRadius;

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

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




        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        wheels[0] = robot.frontLeft;
        wheels[1] = robot.frontRight; // neg
        wheels[2] = robot.backLeft; // neg
        wheels[3] = robot.backRight;



        for (DcMotor wheel : wheels)
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Not Broken Yet");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetAngle();
        // run until the end of the match (driver presses STOP)
        robot.flywheel.setPower(.55);
        robot.wobble.setPosition(.85);
        //7drive(-63,50,.65);
//turn(0, .5,.4);
        /*fire(1); //Fire 2 rings into the top goal
        sleep(2000);
        fire(2);

        drive(-10,5,.4); //Because my code misinterprets -10 in as
                         // +2 inches, this line moves the robot up a tad
        //My code really does not like negative numbers
        */
        /*robot.wobble.setPosition(.4);
        sleep(1000);
        robot.wobble.setPosition(.85);
        sleep(1000);*/
        //Get in position for the first power shot
        strafeRight(36,50,.4);
        sleep(250);
        strafeRight(40,50,.4);
        fire(1);
        sleep(200);
        //second power shot
        strafeLeft(7.5,15,.4);
        fire(1);
        //feed third ring into our launching mechanism
        //robot.intake.setPower(1);
        sleep(250);
        //robot.intake.setPower(0);
        strafeLeft(7.5,15,.4);
        fire(1);
        robot.flywheel.setPower(.635);

//Position the wobble goal into the first square
        strafeLeft(12, 15,.4);
        robot.wobble.setPosition(.385);
        sleep(500);

    }

    public void drive(double distance, double precision, double maxPower){
        double encoderTickstoTarget = (distance/wheelCircumference)*encoderTicksPerRev;
        //Scale constants to target distance
        double kp = maxPower/(encoderTickstoTarget);
        double kd = 2.5*maxPower/(encoderTickstoTarget);
        for(DcMotor wheel:wheels) {
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheel.setTargetPosition((int) (encoderTickstoTarget));
        }
        double [] prevErr = new double[4];
        double avgError = precision+1;
        int precisionTime = 0;
        while(precisionTime<25&&opModeIsActive()){
            if(Math.abs(avgError) < precision) precisionTime++;
            else precisionTime=0;

            telemetry.addData("Target Position",encoderTickstoTarget);
            telemetry.addData("Error",avgError);
            for(int i = 0; i < wheels.length; i++){ //Use a PD motor control system to direct the motor
                double error = (wheels[i].getCurrentPosition() - wheels[i].getTargetPosition());
                avgError+=error;
                prevErr[i] = error;
                //Display data per wheel
                telemetry.addData((i>=2? "front":"back")+(i%2==0? "Left":"Right")+" Position", wheels[i].getCurrentPosition());
                telemetry.addData((i>=2? "front":"back")+(i%2==0? "Left":"Right")+" Error", error);
                double deltaErr = prevErr[i]-error;
                wheels[i].setPower(error*kp+deltaErr*kd);
            }
            avgError/=4;
            telemetry.addData("Average Error",avgError);
            telemetry.update();
            sleep(10);
        }
        for(DcMotor wheel:wheels) {
            wheel.setPower(0);
        }
    }

    public void strafeRight(double distance, double precision, double maxPower){
        //Convert distance to encoder ticks. Robot moves Circumference/sqrt(2) per revolution
        double encoderTickstoTarget = (Math.sqrt(2)*distance/wheelCircumference)*encoderTicksPerRev;
        //Scale constants to the target distance
        double kp = maxPower/(encoderTickstoTarget);
        double kd = maxPower/(encoderTickstoTarget);
        for(int i = 0; i < wheels.length; i++) {
            wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheels[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int k = 1;
            //Front left and back right need to be negative to strafe right
            if(i == 0 || i ==3) k *= -1;
            wheels[i].setTargetPosition((int) (k*encoderTickstoTarget));
        }
        double [] prevErr = new double[4];
        double avgError = precision+1;
        int precisionTime = 0;
        while(precisionTime<25 && opModeIsActive()){
            if(Math.abs(avgError) < precision) precisionTime++; //Prevent oscillation by forcing the code to hold the position for 25 loops (.25 s)
            else precisionTime=0;
            avgError = 0;
            double correction = checkDirection();
            for(int i = 0; i < wheels.length; i++){ //Use a proportional motor control system to direct the motor
                double error = (wheels[i].getCurrentPosition()-wheels[i].getTargetPosition());
                avgError+=error;
                prevErr[i] = error;
                double deltaErr = prevErr[i]-error;
                double c = 1+(i%2==0? correction:-correction);
                telemetry.addData((i>=2? "front":"back")+(i%2==0? "Left":"Right")+" Position", wheels[i].getCurrentPosition());
                telemetry.addData((i>=2? "front":"back")+(i%2==0? "Left":"Right")+" Error", error);
                wheels[i].setPower(error*kp*c+deltaErr*kd);
            }
            avgError /= 4;
            telemetry.update();
            sleep(10);
        }
        for(DcMotor wheel:wheels) {
            wheel.setPower(0);
        }
    }

    public void strafeLeft(double distance, double precision, double maxPower){
        double encoderTickstoTarget = (Math.sqrt(2)*distance/wheelCircumference)*encoderTicksPerRev;
        double kp = maxPower/(encoderTickstoTarget);
        double kd = maxPower/(encoderTickstoTarget);
        for(int i = 0; i < wheels.length; i++) {
            wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheels[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int k = 1;
            if(i == 0 || i ==3) k *= -1;
            wheels[i].setTargetPosition((int) (k*encoderTickstoTarget));
        }
        double [] prevErr = new double[4];
        double avgError = precision+1;
        int precisionTime = 0;
        while(precisionTime<25&&opModeIsActive()){
            if(Math.abs(avgError) < precision) precisionTime++;
            else precisionTime=0;
            avgError = 0;
            double correction = checkDirection();
            for(int i = 0; i < wheels.length; i++){ //Use a proportional motor control system to direct the motor
                double error = (wheels[i].getTargetPosition()-wheels[i].getCurrentPosition());
                avgError+=error;
                prevErr[i] = error;
                double deltaErr = prevErr[i]-error;
                double c = 1+(i%2==0? correction:-correction);
                telemetry.addData((i>=2? "front":"back")+(i%2==0? "Left":"Right")+" Position", wheels[i].getCurrentPosition());
                telemetry.addData((i>=2? "front":"back")+(i%2==0? "Left":"Right")+" Error", error);
                wheels[i].setPower(error*kp*c+deltaErr*kd);
            }
            avgError /= 4;
            telemetry.update();
            sleep(10);
        }
        for(DcMotor wheel:wheels) {
            wheel.setPower(0);
        }
    }

    public void turn(double angle, double precision, double maxPower){
        //Uses arc length to determine how far wheels must turn to rotate angle degrees
        double targetAngle = robot.straight.firstAngle + angle - 90;
        for(DcMotor wheel:wheels) {
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        double avgError = precision+1;
        int precisionTime = 0;
        while(precisionTime<25){
            if(avgError < precision)precisionTime++;
            else precisionTime=0;
            avgError = 0;
            for(int i = 0; i < wheels.length; i++){ //Use a proportional motor control system to direct the motor
                double pCoef = maxPower;
                if(i == 2 || i ==3) pCoef *= -1;
                double error = (angle-getAngle());
                avgError += error;
                wheels[i].setPower((avgError*pCoef));
            }
            avgError /= 4;
            telemetry.update();
            sleep(10);
        }
        for(DcMotor wheel:wheels) {
            wheel.setPower(0);
        }
    }

    public void fire(int ringNumber){
        // Although not currently used for any ringNumber > 1, this loop would fire as many rings
        //as are held at a given time.
        for(int i = 0; i < ringNumber;i++){
            robot.fire.setPower(1);
            sleep(650*ringNumber);
            robot.fire.setPower(0);
        }
    }

    private void resetAngle(){
        //Resets imu current heading to the default
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getRelAngle(){
        //Get angle relative to the robot's heading as of the last resetAngle()

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double getAngle(){
        //Get angle relative to the starting position of the robot, saved upon initialization

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - robot.straight.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle = deltaAngle;

        return globalAngle;
    }


    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .001225;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.
        correction *= gain;
        return correction;
    }
}