package org.firstinspires.ftc.teamcode.opmodes.autonomous.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Config
@Autonomous(name = "Red Autonomous", group = "Testing")
public class RedAutonomous extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(260)
                .build();

        waitForStart();

        telemetry.addData("Made by:", "Mick");
        telemetry.update();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);
//
//        Pose2d poseEstimate = drive.getPoseEstimate();
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }


    public static double GridToInch(double Grids) {
        return Grids * 24;
    }

}
