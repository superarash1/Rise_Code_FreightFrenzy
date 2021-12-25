package org.firstinspires.ftc.teamcode;

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
public class oneMdrive extends LinearOpMode {

    oneMH TIseBot = new oneMH();
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_CM = 537.7;


    @Override
    public void runOpMode() {

        TIseBot.init(hardwareMap);

        TIseBot.cowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TIseBot.cowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TIseBot.cowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Path0", "Starting at %7d", TIseBot.cowMotor.getCurrentPosition());
        telemetry.update();

        telemetry.update();

        //Arm length level
        int armNumber = 1;

        //Specific distance for arm length so that it can retreat back to zero when gamepad1.b and when armNumber == 4
        int armLength = 0;

        waitForStart();


        while (opModeIsActive()){
//          Sets the distance arm will go based on it's current position
            if (armNumber == 1) {
                encoderCows(0.5, 1400);
                //It should be at the first position so it shouldn't have moved anywhere
                armLength = 0;
            }
            else if (armNumber == 2) {
                encoderCows(0.5, 1400);
                //For the armLength variable just add up previous distances
                armLength = 1400;
            }
            else if (armNumber == 3) {
                encoderCows(0.5, 1400);

                armLength = 2800;
            }
            else if (armNumber == 4) {
                encoderCows(0.5, 1400);

                armLength = 3200;
            }

            //Moves it to whatever it is
            if (gamepad1.x && armNumber != 4) {

//              Moves it to the aforementioned distance
                TIseBot.cowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//              Adds to the variable this way it will know when it has reached the end
                armNumber += 1;
            }

            // if the arm is at 4 and x is pressed it will move it back instead of forward
            else if (gamepad1.x && armNumber == 4) {
                //sets it back to the original position
                encoderCows(1.0, -armLength);
                TIseBot.cowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //resets all variables
                armLength = 0;
                armNumber = 1;
            }

            // reset the arm with gamepad1.b, the button to activate can change but for now this is what it is
            if (gamepad1.b) {
                encoderCows(1.0, -armLength);
                TIseBot.cowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armNumber = 1;
                //Just in case it is still set to the armNumber 2 or 3's speed and distance
                encoderCows(0.5, 1400);
            }

            //once it goes past armNumber 3 it goes back to the first position using negative drive
            if (armNumber == 4) {
                //For the negative
                encoderCows(1.0, -armLength);
                TIseBot.cowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //sets it back to arm 1 so that it can be done back again.
                armNumber = 1;
                //makes sure the distance and speed are right just in case it doesn't auto do it
                encoderCows(0.5, 1400);
            }

        }
    }

    public void encoderCows(double speed, double distance) {

        int newWheelTarget;

        if (opModeIsActive()) {
            newWheelTarget = TIseBot.cowMotor.getCurrentPosition() + (int) (distance);

        }
    }
}