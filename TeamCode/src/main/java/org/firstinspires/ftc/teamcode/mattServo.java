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
public class mattServo extends LinearOpMode {
    mattHServo TIsebot = new mattHServo();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double duckSpin = 0;
        boolean open = false;
        int bTurn = 0;

        TIsebot.init(hardwareMap);


        telemetry.addLine("Hi");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                TIsebot.carSpinL.setPower(1.0);
            }
            else {
                TIsebot.carSpinL.setPower(0);
            }

//          TUrning the box
            if (gamepad1.dpad_down) {
//             there are two servos controllering the box turning
                TIsebot.boxT1.setPower(-0.4);
                TIsebot.boxT2.setPower(-0.4);
                telemetry.update();
            }
            else if (gamepad1.dpad_up) {
                TIsebot.boxT1.setPower(0.4);
                TIsebot.boxT2.setPower(0.4);
            }
            else {
                TIsebot.boxT1.setPower(0);
                TIsebot.boxT2.setPower(0);
            }

            if (gamepad1.left_bumper) {
                if (open = false) {
                    TIsebot.boxO.setPosition(75/360);
                    open = true;
                }
                else if (open = true) {
//                  open box so that it will let stuff go
                    TIsebot.boxO.setPosition(45/360);
                    open = false;
                }
            }

            telemetry.addData("Positon of box", TIsebot.boxT1.getPower());
            telemetry.addData("Power of carServo", TIsebot.carSpinL.getPower());
            telemetry.addData("Wanted position of box", 0.1*bTurn);
            telemetry.addData("Position of door", TIsebot.boxO.getPosition());
            telemetry.update();
        }
    }
}