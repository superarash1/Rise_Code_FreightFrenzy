package org.firstinspires.ftc.teamcode.OpModes.TeleOp_OpModes;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.TeleOp.PersonTrackControl;

@TeleOp
public class PersonTracker_TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        PhotonCore.enable();

        PersonTrackControl control = new PersonTrackControl("frontLeft", "frontRight", "backRight", "backLeft", "spinny", hardwareMap, telemetry, gamepad1);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            control.Drive();
            control.Turret();

            telemetry.update();
        }

        control.closeCamera();
    }
}
