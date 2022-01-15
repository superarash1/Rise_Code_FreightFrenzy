package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class mattServo extends LinearOpMode {
    mattHServo TIsebot = new mattHServo();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double duckSpin = 0;

        telemetry.addLine("Hi");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //sets it to auto move to the power of duck spin
            TIsebot.duck1.setPower(0.1*duckSpin);
            TIsebot.duck2.setPower(-0.1*duckSpin);

            if (gamepad1.x) {
                duckSpin =+ 1;
            }
            if (gamepad1.b) {
                duckSpin = 0;
            }

            telemetry.addData("Power of duck1:", 0.1*duckSpin);
            telemetry.addData("Power of duck2:", 0.1*duckSpin);
        }
    }
}
