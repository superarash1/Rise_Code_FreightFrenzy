package org.firstinspires.ftc.teamcode.OldFFCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.OldFFCode.Hardware;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Disabled
public class legitAuton extends LinearOpMode {

    // Define Webcam
    OpenCvCamera webcam;

    // Create pipeline
    static TestPipeline pipeline1;
    static OpenCV_Pipeline pipeline2;

    Hardware TIseBot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    // Constants to find the amount of encoder ticks per CM
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.35;
    static final double WHEEL_DIAMETER_CM = 9.60;

    // Finds the amount of encoder ticks per CM
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);
    static final double ARM_COUNTS_PER_DEGREE = COUNTS_PER_MOTOR_REV / 360;

    public double scoringCycles = 2;

    Orientation lastAngles = new Orientation();
    public double globalAngle;

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

//        TIseBot.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        TIseBot.arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        TIseBot.arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        TIseBot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        TIseBot.arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        TIseBot.arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set up pipeline
        pipeline1 = new TestPipeline();
        pipeline2 = new OpenCV_Pipeline();
        webcam.setPipeline(pipeline1);

        // Start camera streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d", TIseBot.frontLeftMotor.getCurrentPosition(), TIseBot.frontRightMotor.getCurrentPosition(), TIseBot.backRightMotor.getCurrentPosition(), TIseBot.backLeftMotor.getCurrentPosition());

        telemetry.update();

        waitForStart();

        // Telemetry data
        telemetry.addData("Region 1", pipeline1.region1Avg());
        telemetry.addData("Region 2", pipeline1.region2Avg());
        telemetry.addData("Region 3", pipeline1.region3Avg());

        telemetry.update();

        // This compares the light values to determine which region has the highest values to determine the CSE's location
        if ((pipeline1.region1Avg() > pipeline1.region2Avg()) && (pipeline1.region1Avg() > pipeline1.region3Avg())){
            telemetry.addLine("Bottom");
            telemetry.update();

            IMU_Turn_PID_Total(25,1);
            sleep(50);

            // TODO: reset loop thingy
//            Double_Arm_PID(25,1);
//            TIseBot.arm1.setTargetPosition((int) (((int) 25 + (TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE))/10));
//            TIseBot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            TIseBot.arm2.setTargetPosition((int) (((int) 25 + (TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE))/10));
//            TIseBot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(50);

            PIDDrive(1000, 1);
            sleep(50);

        } else if ((pipeline1.region2Avg() > pipeline1.region1Avg()) && (pipeline1.region2Avg() > pipeline1.region3Avg())){
            telemetry.addLine("Middle");
            telemetry.update();

            IMU_Turn_PID_Total(25,1);
            sleep(50);

            // TODO: reset loop thingy
//            Double_Arm_PID(50,1);
//            TIseBot.arm1.setTargetPosition((int) (((int) 50 + (TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE))/10));
//            TIseBot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            TIseBot.arm2.setTargetPosition((int) (((int) 50 + (TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE))/10));
//            TIseBot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(50);

            PIDDrive(1000, 1);
            sleep(50);

        } else if ((pipeline1.region3Avg() > pipeline1.region1Avg()) && (pipeline1.region3Avg() > pipeline1.region2Avg())){
            telemetry.addLine("Top");
            telemetry.update();

            IMU_Turn_PID_Total(25,1);
            sleep(50);

            // TODO: reset loop thingy
//            Double_Arm_PID(75,1);
//            TIseBot.arm1.setTargetPosition((int) (((int) 75 + (TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE))/10));
//            TIseBot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            TIseBot.arm2.setTargetPosition((int) (((int) 75 + (TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE))/10));
//            TIseBot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(50);

            PIDDrive(1000, 1);
            sleep(50);


        }

        for (int i = 1; i < scoringCycles; i++){

            webcam.setPipeline(pipeline2);

            IMU_Turn_PID_Relative(65,1);
            sleep(50);

//            forwardsDistanceSensorDrive(100,0.5);
            sleep(50);

            IMU_Turn_PID_Total(180, 1);
            sleep(50);

            TIseBot.intake.setPower(1);
            PIDDrive(30, 1);
            sleep(50);

            PIDDrive(-30, 1);
            sleep(50);

            IMU_Turn_PID_Relative(90, 1);
            sleep(50);

            // TODO: reset loop thingy
//            Double_Arm_PID(75,1);
//            TIseBot.arm1.setTargetPosition((int) (((int) 75 + (TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE))/10));
//            TIseBot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            TIseBot.arm2.setTargetPosition((int) (((int) 75 + (TIseBot.arm1.getCurrentPosition()*ARM_COUNTS_PER_DEGREE))/10));
//            TIseBot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(50);

//            backwardsDistanceSensorDrive(100, 0.5);
            sleep(50);

            PIDDrive(30, 1);
        }

        // forwardsDistanceSensorDrive(100,0.5);
    }

    // The pipeline
    public static class TestPipeline extends OpenCvPipeline {

        /** Most important section of the code: Colors **/
        static final Scalar GOLD = new Scalar(255, 215, 0);
        static final Scalar CRIMSON = new Scalar(220, 20, 60);
        static final Scalar AQUA = new Scalar(79, 195, 247);
        static final Scalar PARAKEET = new Scalar(3, 192, 74);
        static final Scalar CYAN = new Scalar(0, 139, 139);

        // Create a Mat object that will hold the color data
        Mat HSV = new Mat();
        Mat valueChannel = new Mat();

        // Define the dimensions and location of each region
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(1, 70);
        static final int REGION1_WIDTH = 105;
        static final int REGION1_HEIGHT = 105;
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(107,70);
        static final int REGION2_WIDTH = 105;
        static final int REGION2_HEIGHT = 105;
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(213,70);
        static final int REGION3_WIDTH = 105;
        static final int REGION3_HEIGHT = 105;

        // Create the points that will be used to make the rectangles for the region
        Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);

        Point region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);

        Point region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);

        // Creates a field of type "Mat"
        Mat region1, region2, region3;

        // Creating an array for each region which have an element for each channel of interest
        int region1Value;
        int region2Value;
        int region3Value;

        @Override
        public Mat processFrame(Mat input) {

            // Converts the RGB colors from the video to HSV, which is more useful for image analysis
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV_FULL);

            // Extracts the value channel from HSV
            Core.extractChannel(HSV, valueChannel, 2);

            // Creates the regions and finds the value of each regions
            region1 = valueChannel.submat(new Rect(region1_pointA, region1_pointB));
            region2 = valueChannel.submat(new Rect(region2_pointA, region2_pointB));
            region3 = valueChannel.submat(new Rect(region3_pointA, region3_pointB));

            // Gets the average value for all of the pixels in the region
            region1Value = (int) Core.mean(region1).val[0];
            region2Value = (int) Core.mean(region2).val[0];
            region3Value = (int) Core.mean(region3).val[0];

            // Draws rectangles representing the regions in the camera stream
            Imgproc.rectangle(HSV, region1_pointA, region1_pointB, GOLD,1);
            Imgproc.rectangle(HSV, region2_pointA, region2_pointB, CRIMSON,1);
            Imgproc.rectangle(HSV, region3_pointA, region3_pointB, GOLD,1);

            return HSV;
        }

        // Creating methods with the avgs in order to use them more universally
        public int region1Avg() {
            return region1Value;
        }
        public int region2Avg() {
            return region2Value;
        }
        public int region3Avg() {
            return region3Value;
        }
    }

    public static class OpenCV_Pipeline extends OpenCvPipeline {

        /** Most important section of the code: Colors **/
        static final Scalar GOLD = new Scalar(255, 215, 0);
        static final Scalar CRIMSON = new Scalar(220, 20, 60);
        static final Scalar AQUA = new Scalar(79, 195, 247);
        static final Scalar PARAKEET = new Scalar(3, 192, 74);
        static final Scalar CYAN = new Scalar(0, 139, 139);
        static final Scalar WHITE = new Scalar(255, 255, 255);

        // Create a Mat object that will hold the color data

        Mat Blur = new Mat();

        Mat Hierarchy = new Mat();
        Mat valueThreshold = new Mat();
        Mat circles = new Mat();

        Rect circleRect;

        public int minValueThreshold;
        public int maxValueThreshold;

        List<MatOfPoint> Contours;
        MatOfPoint biggestContour;

        // Make a Constructor
        public OpenCV_Pipeline() {
            minValueThreshold = 20;
            maxValueThreshold = 150;

            Contours = new ArrayList<MatOfPoint>();
        }

        public double rectCheckBottom(MatOfPoint contour) {
            Rect rectInfo = Imgproc.boundingRect(contour).clone();

            double ratio = rectInfo.width/rectInfo.height;

            return ratio / 11;
        }
        public double rectCheckMiddle(MatOfPoint contour) {
            Rect rectInfo = Imgproc.boundingRect(contour).clone();

            double ratio = rectInfo.width/rectInfo.height;

            return ratio / 8;
        }

        public double shippingHubCheck(MatOfPoint contour){
            Rect rectInfo = Imgproc.boundingRect(contour).clone();
            double contourArea = Imgproc.contourArea(contour);

            double value = rectInfo.area()/contourArea;

            return value/5;
        }

        public double rectCheckTop(MatOfPoint contour) {
            Rect rectInfo = Imgproc.boundingRect(contour).clone();

            double ratio = rectInfo.width/rectInfo.height;

            return ratio / 15;
        }

        public double circleCheck(MatOfPoint contour) {

            Rect circleInfo = Imgproc.boundingRect(contour).clone();

            double radius = circleInfo.width/2;
            double area = Math.PI * (radius*radius);

            double idealArea = circleInfo.area()/1.273239545;

            return area / idealArea;
        }

        @Override
        public Mat processFrame(Mat input) {

            Mat gray = new Mat(input.rows(), input.cols(), input.type());

            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

            Mat edges = new Mat(input.rows(), input.cols(), input.type());

            Size kSize = new Size(3, 3);

            Imgproc.blur(gray, edges, kSize);
//
            Imgproc.Canny(edges, edges, minValueThreshold, maxValueThreshold);

            Contours.clear();

            Imgproc.findContours(edges, Contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            Imgproc.drawContours(input, Contours, -1, AQUA); //input

//            MatOfPoint2f thisContour2f = new MatOfPoint2f();
//            MatOfPoint approxContour = new MatOfPoint();
//            MatOfPoint2f approxContour2f = new MatOfPoint2f();
//
//            Rect ret = null;

//            edges.convertTo(thisContour2f, CvType.channels(0));
//
//            Imgproc.approxPolyDP(thisContour2f, approxContour2f, 2, true);
//
//            approxContour2f.convertTo(approxContour, CvType.channels(0));

//            if (approxContour.size().height == 4) {
//                Imgproc.boundingRect(approxContour);
//            }

//            long count = approxContour.total();

            for (int i = 0; i < Contours.size(); i++){
                if ((shippingHubCheck(Contours.get(i)) > 0.8) && (shippingHubCheck(Contours.get(i)) < 1.2)){
                    circleRect = Imgproc.boundingRect(Contours.get(i));
                    Imgproc.rectangle(input, circleRect, PARAKEET, 2); // input
                }

//                if (count == 3) {
//                    circleRect = Imgproc.boundingRect(Contours.get(i));
//                    Imgproc.rectangle(input, circleRect, PARAKEET, 2);
//                }
            }

            /*
            Imgproc.HoughCircles(edges, circles, Imgproc.HOUGH_GRADIENT, Math.PI/180, 50, 1, 50);
            for (int i = 0; i < circles.cols(); i++){
                double[] data = circles.get(0,i);
                Point center = new Point(Math.round(data[0]), Math.round(data[1]));
                Imgproc.circle(HSV, center, 1, new Scalar(0,0,255), 2, 8, 0);

            }             */
//
//            Imgproc.HoughCircles(edges, circles, Imgproc.HOUGH_GRADIENT,  1, 20, 75, 40);
//
//            for (int i = 0; i < circles.cols(); i++ ) {
//                double[] data = circles.get(0, i);
//                Point center = new Point(Math.round(data[0]), Math.round(data[1]));
//                // circle center
//                Imgproc.circle(edges, center, 1, CRIMSON, 4, 8, 0 );
//                // circle outline
//                int radius = (int) Math.round(data[2]);
//                Imgproc.circle(edges, center, radius, CRIMSON, 3, 8, 0 );
//            }

            return input;
        }
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

//    public void Double_Arm_PID(double Degrees, double tolerance) { // TODO: convert spinnerCM to degrees
//
//        double kp = 1;
//        double kd = 0.000;
//        double ki = 0.000;
//
//        // TODO: kGravity
//
//        double kGravity = 0;
//        double kV = 0;
//
//        double referenceVelocity = 0;
//
//        double PID = 0;
//        double power = 0;
//
//        double P = 0;
//
//        double previousError = 0;
//        double error = tolerance + 1;
//
//        double area = 0;
//        double previousArea = 0;
//
//        double dt = 40;
//        double dts = dt/1000;
//
//        while (opModeIsActive()) {
//
//            telemetry.addData("Target Angle: ", TIseBot.arm1.getTargetPosition());
//            telemetry.addData("Current Angle: ", TIseBot.arm1.getCurrentPosition());
//
//            telemetry.addData("Power", power);
//
//            telemetry.addData("de(t)/dt: ", (error - previousError)/dts);
//
//            telemetry.addData("Proportion: ", P);
//
//            telemetry.addData("Error: ", error);
//
//            telemetry.addData("Derivative: ", kd * ((error - previousError)/dts));
//            telemetry.addData("Integral: ", ki * area);
//
//
//            telemetry.addData("Previous Error: ", previousError);
//
//            telemetry.addData("Area: ", area);
//            telemetry.addData("Previous Area: ", previousArea);
//
//            telemetry.addData("dtS", dts);
//
//            telemetry.update();
//
//            previousError = error;
//            error = (TIseBot.arm1.getTargetPosition() - TIseBot.arm1.getCurrentPosition());
//
//            previousArea = area;
//            area = (error * dts) + previousArea;
//
//            P = Math.abs(error)/(Degrees);
//
//            PID = kp * P + kd * ((error - previousError)/dts) + ki * area;
//            power = PID + kGravity * Math.sin(TIseBot.arm1.getTargetPosition()) + kV * (referenceVelocity * Math.signum(PID)); // TODO: potentially change to sine
//
//            TIseBot.arm1.setPower(-power);
//            TIseBot.arm2.setPower(power);
//
//            sleep((long) dt);
//        }
//
//        TIseBot.arm1.setPower(0);
//        TIseBot.arm2.setPower(0);
//
//        // Resets encoders
//        TIseBot.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        TIseBot.arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        TIseBot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        TIseBot.arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }

//    public void forwardsDistanceSensorDrive(double distanceCM, double tolerance){
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            double P = 0;
//
//            double kp = 1.35;
//            double kd = 0.0002; // Constant of derivation
//            double ki = 0.000005;
//
//            double kStatic = 0;
//            double kV = 0;
//
//            double referenceVelocity = 0;
//
//            double dt = 50; //Delta time = 20 ms/cycle
//            double dtS = dt/1000;
//
//            double error;
//            double previousError = 0;
//
//            double area = 0;
//            double previousArea = 0;
//
//            double PID = 0;
//            double power = 0;
//
//            error = tolerance + 1;
//
//            while ((Math.abs(error) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError
//
//                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", distanceCM);
//                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", TIseBot.backDistance.getDistance(DistanceUnit.CM));
//
//                telemetry.addData("DistanceCM: ", (int)(distanceCM * COUNTS_PER_CM));
//
//                telemetry.addData("power: ", power);
//
//                telemetry.addData("Friction FeedForward:", kStatic * Math.signum(PID));
//                telemetry.addData("Velocity FeedForward:", kV * (referenceVelocity * Math.signum(PID)));
//
//                telemetry.addData("Proportion:", P);
//                telemetry.addData("Derivative:", kd * ((error - previousError) / dtS));
//                telemetry.addData("Integral:", ki * area);
//
//                telemetry.addData("de(t)/dt", ((error - previousError) / dtS));
//
//                telemetry.addData("error:", error);
//                telemetry.addData("previous error:", previousError);
//
//                telemetry.addData("∫e(t)dt:", area);
//                telemetry.addData("previous ∫e(t)dt:", previousArea);
//
//                telemetry.addData("dtS", dtS);
//
//                telemetry.update();
//
//                previousError = error;
//
//                error = TIseBot.backDistance.getDistance(DistanceUnit.CM);
//
//                P = Math.abs(error)/(distanceCM);
//
//                previousArea = area;
//
//                area = error * dtS + previousArea;
//
//                PID = kp * P + kd * ((error - previousError) / dtS) + (ki * area);
//
//                power = PID + kStatic * Math.signum(PID) + kV * (referenceVelocity * Math.signum(PID));
//
//                wheels[0].setPower(power);
//                wheels[1].setPower(power);
//                wheels[2].setPower((power)*1.5);
//                wheels[3].setPower((power)*1.5);
//
//                sleep((long) dt);
//            }
//
//            // Stop all motion;
//            for (int i = 0; i < 4; i++){
//                wheels[i].setPower(0);
//
//                // Resets encoders
//                wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                wheels[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//        }
//    }
//
//    public void backwardsDistanceSensorDrive(double distanceCM, double tolerance){
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            double P = 0;
//
//            double kp = 1.35;
//            double kd = 0.0002; // Constant of derivation
//            double ki = 0.000005;
//
//            double kStatic = 0;
//            double kV = 0;
//
//            double referenceVelocity = 0;
//
//            double dt = 50; //Delta time = 20 ms/cycle
//            double dtS = dt/1000;
//
//            double error;
//            double previousError = 0;
//
//            double area = 0;
//            double previousArea = 0;
//
//            double PID = 0;
//            double power = 0;
//
//            error = tolerance + 1;
//
//            while ((Math.abs(error) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError
//
//                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", distanceCM);
//                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", TIseBot.backDistance.getDistance(DistanceUnit.CM));
//
//                telemetry.addData("DistanceCM: ", (int)(distanceCM * COUNTS_PER_CM));
//
//                telemetry.addData("power: ", power);
//
//                telemetry.addData("Friction FeedForward:", kStatic * Math.signum(PID));
//                telemetry.addData("Velocity FeedForward:", kV * (referenceVelocity * Math.signum(PID)));
//
//                telemetry.addData("Proportion:", P);
//                telemetry.addData("Derivative:", kd * ((error - previousError) / dtS));
//                telemetry.addData("Integral:", ki * area);
//
//                telemetry.addData("de(t)/dt", ((error - previousError) / dtS));
//
//                telemetry.addData("error:", error);
//                telemetry.addData("previous error:", previousError);
//
//                telemetry.addData("∫e(t)dt:", area);
//                telemetry.addData("previous ∫e(t)dt:", previousArea);
//
//                telemetry.addData("dtS", dtS);
//
//                telemetry.update();
//
//                previousError = error;
//
//                error = distanceCM - TIseBot.backDistance.getDistance(DistanceUnit.CM);
//
//                P = Math.abs(error)/(distanceCM);
//
//                previousArea = area;
//
//                area = error * dtS + previousArea;
//
//                PID = kp * P + kd * ((error - previousError) / dtS) + (ki * area);
//
//                power = PID + kStatic * Math.signum(PID) + kV * (referenceVelocity * Math.signum(PID));
//
//                wheels[0].setPower(power);
//                wheels[1].setPower(power);
//                wheels[2].setPower((power)*1.5);
//                wheels[3].setPower((power)*1.5);
//
//                sleep((long) dt);
//            }
//
//            // Stop all motion;
//            for (int i = 0; i < 4; i++){
//                wheels[i].setPower(0);
//
//                // Resets encoders
//                wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                wheels[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//        }
//    }

    // TODO: Make into avg error and such
    public void PIDDrive(double distanceCM, double tolerance) { // TODO: Adjust Tolerance

        int []newWheelTarget = new int[4];

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            for (int i = 0; i < 4; i++) {
                newWheelTarget[i] = (int)(wheels[0].getCurrentPosition()*COUNTS_PER_CM) + (int)(distanceCM); // TODO: Get avg position
            }

            double[] P = new double[4];

            double kp = 1.35;
            double kd = 0.0002; // Constant of derivation
            double ki = 0.000005;

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


            error[0] = tolerance + 1;

            while ((Math.abs(error[0]) > tolerance) && opModeIsActive()) { // TODO: replace error[0] with avgError

                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newWheelTarget[0], newWheelTarget[1], newWheelTarget[2], newWheelTarget[3]);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", wheels[0].getCurrentPosition()*COUNTS_PER_CM, wheels[1].getCurrentPosition()*COUNTS_PER_CM, wheels[2].getCurrentPosition()*COUNTS_PER_CM, wheels[3].getCurrentPosition()*COUNTS_PER_CM);

                telemetry.addData("DistanceCM: ", distanceCM);

                telemetry.addData("power: ", power[0]);

                telemetry.addData("Friction FeedForward:", kStatic * Math.signum(PID[0]));
                telemetry.addData("Velocity FeedForward:", kV * (referenceVelocity * Math.signum(PID[0])));

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

                error[0] = (int)(distanceCM) - (wheels[0].getCurrentPosition()*COUNTS_PER_CM);
                error[1] = (int)(distanceCM) - wheels[1].getCurrentPosition()*COUNTS_PER_CM;
                error[2] = (int)(distanceCM) - wheels[2].getCurrentPosition()*COUNTS_PER_CM;
                error[3] = (int)(distanceCM) - wheels[3].getCurrentPosition()*COUNTS_PER_CM;

                P[0] = Math.abs(error[0])/(int)(distanceCM);
                P[1] = Math.abs(error[1])/(int)(distanceCM);
                P[2] = Math.abs(error[2])/(int)(distanceCM);
                P[3] = Math.abs(error[3])/(int)(distanceCM);

                previousArea[0] = area[0];
                previousArea[1] = area[1];
                previousArea[2] = area[2];
                previousArea[3] = area[3];

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

                wheels[0].setPower(power[0]);
                wheels[1].setPower(power[1]);
                wheels[2].setPower((power[2])*1.5);
                wheels[3].setPower((power[3])*1.5);

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
            wheels[2].setPower((power)*1.5);
            wheels[3].setPower(-(power)*1.5);

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
            wheels[2].setPower((power)*1.5);
            wheels[3].setPower(-(power)*1.5);

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
                newWheelTarget[i] = (int)(wheels[0].getCurrentPosition()*COUNTS_PER_CM) + (int)(distanceCM); // TODO: Get avg position
            }

            double kp = 1;
            double kd = 0.00015; // Constant of derivation
            double ki = 0.00004;

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

                if ((error[1] * avgArea) < 0){
                    previousArea = 0;
                }

                avgPower = kp * Proportion + kd * Derivative + (ki * avgArea);

                wheels[0].setPower(-avgPower);
                wheels[1].setPower(avgPower);
                wheels[2].setPower(-(avgPower)*1.5);
                wheels[3].setPower((avgPower)*1.5);

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
}
