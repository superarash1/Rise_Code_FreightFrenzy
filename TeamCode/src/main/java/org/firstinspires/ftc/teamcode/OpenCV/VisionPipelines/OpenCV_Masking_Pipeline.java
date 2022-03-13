package org.firstinspires.ftc.teamcode.OpenCV.VisionPipelines;

import static org.opencv.core.Core.inRange;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class OpenCV_Masking_Pipeline extends OpenCvPipeline {

    Telemetry telemetry;

    /** Most important section of the code: Colors **/
    static final Scalar GOLD = new Scalar(255, 215, 0);
    static final Scalar CRIMSON = new Scalar(220, 20, 60);
    static final Scalar AQUA = new Scalar(79, 195, 247);
    static final Scalar PARAKEET = new Scalar(3, 192, 74);
    static final Scalar CYAN = new Scalar(0, 139, 139);
    static final Scalar WHITE = new Scalar(255, 255, 255);

    // Create a Mat object that will hold the color data

    public enum barcodeReader {
        LEFT,
        MIDDLE,
        RIGHT,
    }

    barcodeReader reader;
    barcodeReader readingAtStart;

    Rect yellowMask;
    Rect whiteMask;

    List<MatOfPoint> yellowContours;
    List<MatOfPoint> whiteContours;

    // Make a Constructor
    public OpenCV_Masking_Pipeline(Telemetry telemetry) {
        yellowContours = new ArrayList<MatOfPoint>();
        whiteContours = new ArrayList<MatOfPoint>();

        this.telemetry = telemetry;
    }

    public boolean filterContours(MatOfPoint contour) {
        return Imgproc.contourArea(contour) > 200;
    }

    @Override
    public Mat processFrame(Mat input) {

        Mat HSV = new Mat(input.rows(), input.cols(), input.type());

        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

        Scalar lowYellow = new Scalar(15, 10, 10); // 10, 50, 50
        Scalar highYellow = new Scalar(22, 255, 255); //25, 255, 255
        Mat maskYellow = new Mat();

        Scalar lowWhite = new Scalar(90, 15, 110);
        Scalar highWhite = new Scalar(140, 40, 255);
        Mat maskWhite = new Mat();

        inRange(HSV, lowYellow, highYellow, maskYellow);
        inRange(HSV, lowWhite, highWhite, maskWhite);

        yellowContours.clear();
        whiteContours.clear();
//        Imgproc.Canny(maskYellow, maskYellow, 0, 250);
//        Imgproc.Canny(maskWhite, maskWhite, 20, 100);

        Mat yellowBlur = new Mat();

//        Imgproc.Laplacian(maskYellow, yellowLaplace, 10);

//        Imgproc.bilateralFilter(maskYellow, yellowBlur, 15, 80, 80, Core.BORDER_DEFAULT);
        Imgproc.medianBlur(maskYellow, yellowBlur, 15);

        Imgproc.adaptiveThreshold(yellowBlur, maskYellow, 125, Imgproc.ADAPTIVE_THRESH_MEAN_C,
                Imgproc.THRESH_BINARY, 11, 12);

        Imgproc.findContours(maskYellow, yellowContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, yellowContours, -1, CRIMSON); //input

        Imgproc.findContours(maskWhite, whiteContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, whiteContours, -1, AQUA); //input

        for (int i = 0; i < yellowContours.size(); i++){
            if (filterContours(yellowContours.get(i))){
                yellowMask = Imgproc.boundingRect(yellowContours.get(i));
                Imgproc.rectangle(input, yellowMask, CRIMSON, 2);
            }
        }

        for (int i = 0; i < whiteContours.size(); i++){
            if (filterContours(whiteContours.get(i))){
                whiteMask = Imgproc.boundingRect(whiteContours.get(i));
                Imgproc.rectangle(input, whiteMask, AQUA, 2);
            }
        }

//        if (yellowMask.x < (input.cols()/3)) reader = barcodeReader.LEFT;
//        if (yellowMask.x > (input.cols()/3) && yellowMask.x < 2*(input.cols()/3)) reader = barcodeReader.MIDDLE;
//        if (yellowMask.x > 2*(input.cols()/3)) reader = barcodeReader.RIGHT;

        telemetry.addData("Barcode Reading", reader);
        telemetry.update();

        return yellowBlur;
    }

    public void setReadingAtStart(){
        this.readingAtStart = reader;
    }

    public barcodeReader getReading() {
        return this.reader;
    }

    public barcodeReader getReadingAtStart() {
        return this.readingAtStart;
    }
}
