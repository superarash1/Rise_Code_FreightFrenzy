package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.VisionPipelines.OpenCV_Masking_Pipeline;
import org.firstinspires.ftc.teamcode.OpenCV.VisionPipelines.RegionsPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RiseAuton_FF extends LinearOpMode {

    OpenCvCamera webcam;

    // Create Pipeline
    static RegionsPipeline pipeline;

    private ElapsedTime runtime = new ElapsedTime();

    boolean left;
    boolean middle;
    boolean right;

    public int auton = 1;

    @Override
    public void runOpMode() {
        MecanumDriveTrain chassis = new MecanumDriveTrain("frontLeft", "frontRight", "backRight", "backLeft",hardwareMap, telemetry, gamepad1);
        BotMechanisms mechanisms = new BotMechanisms("intake", "armLeft", "armRight",hardwareMap, telemetry, gamepad1);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set up pipeline
        pipeline = new RegionsPipeline();
        webcam.setPipeline(pipeline);

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

//        pipeline.setReadingAtStart();



        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        if ((pipeline.HSV_Value_1[0] < pipeline.HSV_Value_2[0]) && (pipeline.HSV_Value_1[0] < pipeline.HSV_Value_3[0])){
            left = true;
            telemetry.addLine("Left");


        } else if ((pipeline.HSV_Value_2[0] < pipeline.HSV_Value_1[0]) && (pipeline.HSV_Value_2[0] < pipeline.HSV_Value_3[0])){
            middle = true;
            telemetry.addLine("Mid");


        } else if ((pipeline.HSV_Value_3[0] < pipeline.HSV_Value_1[0]) && (pipeline.HSV_Value_3[0] < pipeline.HSV_Value_2[0])){
            right = true;
            telemetry.addLine("Right");

        }
        telemetry.update();

        while (opModeIsActive()){

            telemetry.addData("auton", auton);
            telemetry.update();
            mechanisms.Arm();
            mechanisms.cageRotation();

//            if (auton == 1){
//                if (pipeline.getReadingAtStart() == OpenCV_Masking_Pipeline.barcodeReader.LEFT){
//                    mechanisms.targetPos = 144;
//                } else if (pipeline.getReadingAtStart() == OpenCV_Masking_Pipeline.barcodeReader.MIDDLE){
//                    mechanisms.targetPos = 175;
//                } else if (pipeline.getReadingAtStart() == OpenCV_Masking_Pipeline.barcodeReader.RIGHT){
//                    mechanisms.targetPos = 205;
//                }
//                mechanisms.boxTarget = 0;
//                auton++;
//            }

            if (auton == 1){
                if (left){
                    mechanisms.targetPos = 144;
                    mechanisms.boxTarget = 0;
                } else if (middle){
                    mechanisms.targetPos = 175;
                    mechanisms.boxTarget = 0;
                } else if (right){
                    mechanisms.targetPos = 205;
                    mechanisms.boxTarget = 0;
                }

                auton++;
            }

            if (auton == 2){
                runtime.reset();
                while (runtime.seconds() < 1){

                }
                chassis.PID_Turn(45, 1);
                auton++;
            }
//
//            if (auton == 3){
//                sleep(50);
//                chassis.PID_Drive(50, 1);
//                auton++;
//            }
//
//            if (auton == 4){
//                mechanisms.setGatePosition(0.3);
//                auton++;
//            }
//
//
//            if (auton == 5){
//                sleep(1000);
//                mechanisms.targetPos = -34;
//                mechanisms.boxTarget = 0.7;
//                auton++;
//            }
//
//            if (auton == 6){
//                chassis.PID_Turn(-90, 1);
//                auton++;
//            }
//
//            if (auton == 7){
//                sleep(50);
//                chassis.PID_Strafe(15, 1);
//                auton++;
//            }
//
//            if (auton == 8){
//                sleep(50);
//                chassis.PID_Drive(30, 1);
//                auton++;
//            }
//
//            if (auton == 9){
//                runtime.reset();
//                while (runtime.seconds() < 3){
//                    mechanisms.carouselSetPower(0.6);
//                }
//                mechanisms.carouselSetPower(0);
//                auton++;
//            }
//
//            if (auton == 10){ //
//                chassis.PID_Strafe(10, 1);
//                auton++;
//            }
//
//            if (auton == 11){
//                chassis.PID_Turn(90, 1);
//                auton++;
//            }
//
//            if (auton == 12){
//                chassis.PID_Drive(100, 1);
//                auton++;
//            }
        }
    }
}
