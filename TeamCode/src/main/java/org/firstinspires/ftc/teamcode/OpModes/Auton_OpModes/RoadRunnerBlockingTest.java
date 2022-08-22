package org.firstinspires.ftc.teamcode.OpModes.Auton_OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.BotMechanisms.MecanumDriveTrain;

@Autonomous
public class RoadRunnerBlockingTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDriveTrain drive = new MecanumDriveTrain("frontLeft", "frontRight", "backRight", "backLeft", hardwareMap);

        Pose2d globalPose = new Pose2d(0,0,0);

        drive.setPoseEstimate(globalPose);

        Trajectory trajectoryOne = drive.trajectoryBuilder(globalPose).splineToConstantHeading(new Vector2d(10, 22), 0).build();
//        Trajectory trajectoryTwo = drive.trajectoryBuilder(new Pose2d()).splineTo(new Vector2d(10, 10), Math.toRadians(-90)).build();
        Trajectory trajectoryTwo = drive.trajectoryBuilder(trajectoryOne.end()).splineTo(new Vector2d(22, 0), -Math.toRadians(90)).build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectoryOne);

        sleep(500);

        drive.followTrajectory(trajectoryTwo);


    }
}
