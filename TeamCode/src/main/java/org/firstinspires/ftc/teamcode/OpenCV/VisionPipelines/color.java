package org.firstinspires.ftc.teamcode.OpenCV.VisionPipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class color extends OpenCvPipeline {

    //private Point SIDE_BOTTOM_LEFT = new Point(0, 211.2);
    //private Point SIDE_TOP_RIGHT = new Point(114.56,249.6);

    Telemetry telemetry;

//    private static final Point REGION1_BOTTOM_LEFT = new Point(35.56 - 11.85,210.73 + 23.41);
//    private static final Point REGION1_TOP_RIGHT = new Point(106.67 - 11.85,257.56 + 35.12 - 17.56);
//    private static final Point REGION2_BOTTOM_LEFT = new Point(248.89, 210.73 + 23.41);
//    private static final Point REGION2_TOP_RIGHT = new Point(296.29,257.56 + 35.12 - 17.56);

//    private static final Point REGION2_BOTTOM_LEFT = new Point(142.22 - 23.7 , 480 - 132 - 36);
//    private static final Point REGION2_TOP_RIGHT = new Point(260.74 - 23.7,480 - 168 - 36);
//    private static final Point REGION1_BOTTOM_LEFT = new Point(474.07 - 23.7  - 82.96, 480 - 132 - 36);
//    private static final Point REGION1_TOP_RIGHT = new Point(533.33 - 23.7  - 82.96,480 - 168 - 36);

    private static final Point REGION1_BOTTOM_LEFT = new Point(20,30);
    private static final Point REGION1_TOP_RIGHT = new Point(50,50);
    private static final Point REGION2_BOTTOM_LEFT = new Point(52, 30);
    private static final Point REGION2_TOP_RIGHT = new Point(100,50);

    public color(Telemetry telemetry){

        this.telemetry = telemetry;
    }

    /*private static final Point REGION1_BOTTOM_LEFT = new Point(0,378.24);
    private static final Point REGION1_TOP_RIGHT = new Point(84.53,402.24);
    private static final Point REGION2_BOTTOM_LEFT = new Point(277.74, 378.24);
    private static final Point REGION2_TOP_RIGHT = new Point(350.19,402.24);*/


    private static final Scalar GREEN =  new Scalar(0, 255, 0); //RGB

    //Mat Side;
    Mat Region1, Region2;
    //Mat Color = new Mat();
    //Mat HLS = new Mat();
    //int avgSide;

    //private int avg1, avg2;
    private static Scalar avg1, avg2;
    private static int barcode;
    public static int[] YCrCb1 = new int[3];
    public static int[] YCrCb2 = new int[3];

    Mat YCrCb = new Mat();
    //public enum Barcode {
    //    ONE, TWO, THREE
    //}
    //private Barcode number = Barcode.THREE;
    //private volatile Barcode number = Barcode.THREE;

    //convert mat into color depending on scale
   /* public void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2HSV_FULL);
        //vCore.extractChannel(YCrCb, Color, 1);
    }*/
/*
    @Override
    public void init(Mat firstFrame) {
        //inputToCb(firstFrame);

        //Side = Color.submat(new Rect(SIDE_BOTTOM_LEFT,SIDE_TOP_RIGHT));
        Region1 = Color.submat(new Rect(REGION1_BOTTOM_LEFT,REGION1_TOP_RIGHT));
        Region2 = Color.submat(new Rect(REGION2_BOTTOM_LEFT,REGION2_TOP_RIGHT));

    }
    */

    @Override
    public Mat processFrame(Mat input) {

        //inputToCb(input);
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2HLS);

        Region1 = YCrCb.submat(new Rect(REGION1_BOTTOM_LEFT,REGION1_TOP_RIGHT));
        Region2 = YCrCb.submat(new Rect(REGION2_BOTTOM_LEFT,REGION2_TOP_RIGHT));
        //avgSide = (int) Core.mean(Side).val[0];
        //avg1 = (int) Core.mean(Region1).val[0];
        // avg2 = (int) Core.mean(Region2).val[0];

        avg1 = Core.mean(Region1);
        avg2 = Core.mean(Region2);

        //find max color value for both side + barcodes
        //test color space for biggest contrast between
        //grey + Red/blue (side)
        //blue/red + shipping element color (barcode)
        //need a range for the same color incase element is on third barcode (outside of cam view)

            /*Imgproc.rectangle(
                    input, // Buffer to draw on
                    SIDE_BOTTOM_LEFT, // First point which defines the rectangle
                    SIDE_TOP_RIGHT, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    2 // Thickness of the rectangle lines
            );*/

        Imgproc.rectangle(
                input,
                REGION1_BOTTOM_LEFT,
                REGION1_TOP_RIGHT,
                GREEN,
                2
        );

        Imgproc.rectangle(
                input,
                REGION2_BOTTOM_LEFT,
                REGION2_TOP_RIGHT,
                GREEN,
                2
        );

        findBarcode();

        telemetry.addData("findBarcode", barcode);

        return input;
    }
    //determine if colors are same
    //public boolean sameColor(Scalar one, Scalar two) {

    //}

    //determine which barcode has the element
    public static void findBarcode() {
        if (Math.abs(avg1.val[0] - avg2.val[0]) <= 50) {
            barcode = 3;
        }
        else if (avg2.val[0] > avg1.val[0]) {
            barcode = 1;
        }
        else if (avg1.val[0] > avg2.val[0]) {
            barcode = 2;
        }
    }


    //return barcode constant
    public static int getBarcode() {
        return barcode;
    }

    //get methods for average color values
    public Scalar getAvg1() {
        return avg1;
    }

    public Scalar getAvg2() {
        return avg2;
    }



}