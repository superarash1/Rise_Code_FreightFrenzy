package org.firstinspires.ftc.teamcode.Hardware;

import static org.opencv.core.Core.getOptimalDFTSize;
import static org.opencv.core.Core.inRange;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class DetectionPipeline extends OpenCvPipeline {

    // Creating a Telemetry object to allow us to use the "Telemetry" class without extending to LinearOpMode

    /** Most important section of the code: Colors **/
    // These are just color constants used to define the color for each bounding box we create for the contours
    static final Scalar BLAZE =  new Scalar(255, 102, 0);
    static final Scalar GOLD = new Scalar(255, 215, 0);
    static final Scalar PARAKEET = new Scalar(3, 192, 74);
    static final Scalar CYAN = new Scalar(0, 139, 139);
    static final Scalar CRIMSON = new Scalar(220, 20, 60);
    static final Scalar HOT_PINK = new Scalar(196, 23, 112);
    static final Scalar LIGHT_GRAY = new Scalar(255, 255, 240);
    static final Scalar DARK_GRAY = new Scalar(120,120,120);

    // These are the boolean toggles for each color the code will track
    boolean ORANGE = false;
    boolean YELLOW = false;
    boolean GREEN = false;
    boolean BLUE = true;
    boolean RED = false;
    boolean PINK = false;
    boolean BLACK = false;
    boolean WHITE = false;

    // These are int variables that keep track of how many contours of a specific color there are

   public int yellowContourCount = 0;

    // Creating lists for all the bounding Rects of each color

    List<Rect> yellowRect;

    // Creating lists for all the MatOfPoints of each contour

    List<MatOfPoint> yellowContours = new ArrayList<>();
    enum Stage {//color difference. greyscale
        detection,//includes outlines
        THRESHOLD,//b&w
        RAW_IMAGE,//displays raw view
    }
    private Stage stageToRenderToViewport = Stage.detection;
    private Stage[] stages = Stage.values();

    public static int rows = 176;
    public static int cols = 144;

    // Define the dimensions and location of each region
    public static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, cols/2-rows/6);
    public static final int REGION1_WIDTH = rows/3;
    public static final int REGION1_HEIGHT = rows/3;
    public static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(rows/3, cols/2-rows/6);
    public static final int REGION2_WIDTH = rows/3;
    public static final int REGION2_HEIGHT = rows/3;
    public static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point((2*rows)/3, cols/2-rows/6);
    public static final int REGION3_WIDTH = rows/3;
    public static final int REGION3_HEIGHT = rows/3;


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
    public static int avg1, avg2, avg3;

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
    // Make a Constructor
    public DetectionPipeline() {

        // Creates an arraylist for each contour list
         yellowContours = new ArrayList<MatOfPoint>();
        // Creates an arraylist for bounding Rect list
     yellowRect = new ArrayList<Rect>();


    }
    public List<Rect> getYellowRect() {

        return yellowRect;
    }
    // Filters the contours to be greater than a specific area in order to be tracked
    public boolean filterContours(MatOfPoint contour) {
        return Imgproc.contourArea(contour) > 50;
    }


    Mat yCbCrChan2Mat = new Mat();
    Mat thresholdMat = new Mat();
    Mat all = new Mat();

    @Override
    public Mat processFrame(Mat input) {

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
        Imgproc.rectangle(input, region2_pointA, region2_pointB, HOT_PINK, 2);
        Imgproc.rectangle(input, region3_pointA, region3_pointB, CRIMSON, 2);

        yellowRect.clear();
        yellowContours.clear();

        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

        //b&w
        Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

        //outline/contour
        //Imgproc.findContours(thresholdMat, yellowContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);


            // Finds the contours and draws them on the screen
            Imgproc.findContours(thresholdMat, yellowContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object

            Imgproc.drawContours(all, yellowContours, -1, GOLD); //input

            // Iterates through each contour
            for (int i = 0; i < yellowContours.size(); i++){

                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(yellowContours.get(i))){
                    // Creates a bounding rect around each contourand the draws it

                    yellowRect.add(Imgproc.boundingRect(yellowContours.get(i)));
                    Imgproc.rectangle(all, yellowRect.get(yellowContourCount), GOLD, 2);

                    // Creates a count for the amount of yellow contours on the the screen
                    yellowContourCount++;
                }
            }
        yellowContourCount = 0;

        // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)


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
