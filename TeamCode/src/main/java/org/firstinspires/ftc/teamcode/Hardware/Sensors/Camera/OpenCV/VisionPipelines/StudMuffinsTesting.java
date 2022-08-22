package org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera.OpenCV.VisionPipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class StudMuffinsTesting extends OpenCvPipeline {

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = .6f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private static int rows = 144;
    private static int cols = 176;

    List<Rect> yellowRect;
    int yellowContourCount = 0;

    Telemetry telemetry;

    public StudMuffinsTesting(Telemetry telemetry){

        this.telemetry = telemetry;
    }

    Mat yCbCrChan2Mat = new Mat();
    Mat thresholdMat = new Mat();
    Mat all = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();

    enum Stage {//color difference. greyscale
        detection,//includes outlines
        THRESHOLD,//b&w
        RAW_IMAGE,//displays raw view
    }

    private Stage stageToRenderToViewport = Stage.detection;
    private Stage[] stages = Stage.values();

    static final Scalar CRIMSON = new Scalar(220, 20, 60);
    static final Scalar AQUA = new Scalar(79, 195, 247);
    static final Scalar PARAKEET = new Scalar(3, 192, 74);
    static final Scalar GOLD = new Scalar(255, 215, 0);
    static final Scalar CYAN = new Scalar(0, 139, 139);

    // Define the dimensions and location of each region
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, rows/2-cols/6);
    static final int REGION1_WIDTH = cols/3;
    static final int REGION1_HEIGHT = cols/3;
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(cols/3, rows/2-cols/6);
    static final int REGION2_WIDTH = cols/3;
    static final int REGION2_HEIGHT = cols/3;
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point((2*cols)/3, rows/2-cols/6);
    static final int REGION3_WIDTH = cols/3;
    static final int REGION3_HEIGHT = cols/3;


    // Create the points that will be used to make the rectangles for the region
    Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);

    Point region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);

    Point region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);

    // Create fields to store the color value information
    Mat region1_G, region2_G, region3_G;

    // Define objects for color spaces
    Mat HLS = new Mat();
    Mat L = new Mat();
    int avg1, avg2, avg3;

    @Override
    public void onViewportTapped() {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int currentStageNum = stageToRenderToViewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }

        stageToRenderToViewport = stages[nextStageNum];
    }

    // Filters the contours to be greater than a specific area in order to be tracked
    public boolean filterContours(MatOfPoint contour) {
        return Imgproc.contourArea(contour) > 100;
    }


    @Override
    public Mat processFrame(Mat input) {

        telemetry.clearAll();

        Imgproc.cvtColor(input, HLS, Imgproc.COLOR_RGB2HLS);
        Core.extractChannel(HLS, L, 1);

        // Create regions, find the color values, and update the fields
        region1_G = L.submat(new Rect(region1_pointA, region1_pointB));
        region2_G = L.submat(new Rect(region2_pointA, region2_pointB));
        region3_G = L.submat(new Rect(region3_pointA, region3_pointB));

        // Finds the avg color value in a region
        avg1 = (int) Core.mean(region1_G).val[0];
        avg2 = (int) Core.mean(region2_G).val[0];
        avg3 = (int) Core.mean(region3_G).val[0];

        // Draws a rectangle on the camera stream and is wha the color constants are used for
        Imgproc.rectangle(input, region1_pointA, region1_pointB, CRIMSON, 2);
        Imgproc.rectangle(input, region2_pointA, region2_pointB, AQUA, 2);
        Imgproc.rectangle(input, region3_pointA, region3_pointB, PARAKEET, 2);

        contoursList.clear();

        /*
         * This pipeline finds the contours of yellow blobs such as the Gold Mineral
         * from the Rover Ruckus game.
         */

        //color diff cb.
        //lower cb = more blue = skystone = white
        //higher cb = less blue = yellow stone = grey
        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

        //b&w
        Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

        //outline/contour
        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        yCbCrChan2Mat.copyTo(all);//copies mat object
        Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


        //get values from frame
        double[] pixMid = thresholdMat.get((int) (input.rows() * midPos[1]), (int) (input.cols() * midPos[0]));//gets value at circle
        valMid = (int) pixMid[0];

        double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]), (int) (input.cols() * leftPos[0]));//gets value at circle
        valLeft = (int) pixLeft[0];

        double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]), (int) (input.cols() * rightPos[0]));//gets value at circle
        valRight = (int) pixRight[0];

        //create three points
        Point pointMid = new Point((int) (input.cols() * midPos[0]), (int) (input.rows() * midPos[1]));
        Point pointLeft = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows() * leftPos[1]));
        Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows() * rightPos[1]));

        //draw circles on those points
        Imgproc.circle(all, pointMid, 5, new Scalar(255, 0, 0), 1);//draws circle
        Imgproc.circle(all, pointLeft, 5, new Scalar(255, 0, 0), 1);//draws circle
        Imgproc.circle(all, pointRight, 5, new Scalar(255, 0, 0), 1);//draws circle

        //draw 3 rectangles
        Imgproc.rectangle(//1-3
                all,
                new Point(
                        input.cols() * (leftPos[0] - rectWidth / 2),
                        input.rows() * (leftPos[1] - rectHeight / 2)),
                new Point(
                        input.cols() * (leftPos[0] + rectWidth / 2),
                        input.rows() * (leftPos[1] + rectHeight / 2)),
                new Scalar(0, 255, 0), 3);
        Imgproc.rectangle(//3-5
                all,
                new Point(
                        input.cols() * (midPos[0] - rectWidth / 2),
                        input.rows() * (midPos[1] - rectHeight / 2)),
                new Point(
                        input.cols() * (midPos[0] + rectWidth / 2),
                        input.rows() * (midPos[1] + rectHeight / 2)),
                new Scalar(0, 255, 0), 3);
        Imgproc.rectangle(//5-7
                all,
                new Point(
                        input.cols() * (rightPos[0] - rectWidth / 2),
                        input.rows() * (rightPos[1] - rectHeight / 2)),
                new Point(
                        input.cols() * (rightPos[0] + rectWidth / 2),
                        input.rows() * (rightPos[1] + rectHeight / 2)),
                new Scalar(0, 255, 0), 3);

        telemetry.addData("valMid", valMid);
        telemetry.addData("valLeft", valLeft);
        telemetry.addData("valRight", valRight);

        telemetry.addData("avg1", avg1);
        telemetry.addData("avg2", avg2);
        telemetry.addData("avg3", avg3);
        telemetry.update();

        yellowContourCount = 0;

        switch (stageToRenderToViewport) {
            case THRESHOLD: {
                return thresholdMat;
            }

            case detection: {
                return all;
            }

            case RAW_IMAGE: {
                return input;
            }

            default: {
                return input;
            }
        }
    }
}
