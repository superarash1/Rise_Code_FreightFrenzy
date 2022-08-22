package org.firstinspires.ftc.teamcode.Control.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera.OpenCV.VisionPipelines.BlockDetection;
import org.firstinspires.ftc.teamcode.Control.MotionControl.PIDF_Controller;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.InertialMeasurementUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class TurretBotTeleOp_Control {

    OpenCvCamera webcam;

    ElapsedTime runtime  = new ElapsedTime();

    Telemetry telemetry;

    Turret turret;
    MecanumDriveTrain driveTrain;
    InertialMeasurementUnit inertialMeasurementUnit;

    Gamepad gamepad1;

    // Create Pipeline
    public static BlockDetection pipeline;

    double boxPositionX;
    boolean lastToggleX = true;

    double power;

    double drive;
    double turn;
    double strafe;
    double fLeft;
    double fRight;
    double bLeft;
    double bRight;
    double max;

    double deltaTime;

    double kF = 0.7;

    public enum turretState {
        MANUAL,
        TRACKING,
        FIXED
    }

    public turretState TurretState;

    public PIDF_Controller TurretPID;
    public PIDF_Controller TurretStationPID;

    public TurretBotTeleOp_Control(String flName, String frName, String brName, String blName, String turretName, HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry){

        TurretPID = new PIDF_Controller(1, 0.00001, telemetry);
        TurretStationPID = new PIDF_Controller(0.8, 0, telemetry);

        inertialMeasurementUnit = new InertialMeasurementUnit(hardwareMap);

        driveTrain = new MecanumDriveTrain(flName, frName, brName, blName, hardwareMap);

        driveTrain.setBreakMode();
        driveTrain.reset();

        turret = new Turret(turretName, hardwareMap);

        // Set up webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set up pipeline
        pipeline = new BlockDetection(telemetry);
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

        TurretPID.tolerance = 0.5;

        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;

        TurretState = turretState.MANUAL;
    }

    public void teleOpDrive(){
        drive = Math.pow(gamepad1.left_stick_y, 3); //Between -1 and 1
        turn = Math.pow(gamepad1.right_stick_x, 3);
        strafe = Math.pow(gamepad1.left_stick_x, 3);

        // Mecanum Drive Calculations
        fLeft = -0.875 * drive + 1 * strafe + 0.8 * turn;
        fRight = -0.875 * drive - 1 * strafe - 0.8 * turn;
        bRight = -0.875 * drive + 1 * strafe - 0.8 * turn;
        bLeft = -0.875 * drive - 1 * strafe + 0.8 * turn;

        // This ensures that the power values the motors are set to are in the range (-1, 1)
        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }



        driveTrain.setPower(fLeft, fRight, bRight, bLeft);
    }

    public void turretSpin(){

        switch (TurretState){
            case MANUAL:
                power = Math.pow(gamepad1.right_trigger - gamepad1.left_trigger, 3);

                if (gamepad1.x != lastToggleX && gamepad1.x){
                    TurretState = turretState.TRACKING;
                }

                lastToggleX = gamepad1.x;

                break;
            case TRACKING:
                if (!pipeline.YellowRect.empty()){
                    boxPositionX = pipeline.YellowRect.x + (pipeline.YellowRect.width/2);

                    power = -TurretPID.PIDF_Power(boxPositionX, 160, 320);
                }

                if (gamepad1.x != lastToggleX && gamepad1.x){
                    TurretState = turretState.MANUAL;
                }

                lastToggleX = gamepad1.x;

                break;

            case FIXED:
                power = TurretPID.PIDF_Power(turret.TurretAngle() + inertialMeasurementUnit.Angle_FieldCentric(), 0, 180) - turn * kF;
        }

        turret.turretMotor.setPower(power);
    }

    public void Telemetry(){
        deltaTime = runtime.seconds();

        telemetry.addData("Power", power);
        telemetry.addData("Proportion", TurretPID.P);
        telemetry.addData("Derivative", TurretPID.D);
        telemetry.addData("delta time", deltaTime);
        telemetry.addData("Turret Position",turret.TurretAngle() + inertialMeasurementUnit.Angle_FieldCentric());
        runtime.reset();
    }

    public void closeCamera(){
        webcam.closeCameraDevice();
    }
}
