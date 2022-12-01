package org.firstinspires.ftc.teamcode.opmodes.autonomous.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.teleop.testing.SlideTest;
import org.firstinspires.ftc.teamcode.systems.Controllers.intake.Intake;
import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.SlideHeight;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Config
@Autonomous(name = "Blue Autonomous", group = "Testing")
public class BlueAutonomous extends LinearOpMode {
    Intake intake;
    LinearSlide slide;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        slide = new LinearSlide(hardwareMap);

        waitForStart();

        telemetry.addData("Made by:", "Mick");
        telemetry.update();

        if (isStopRequested()) return;

//        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
//                .strafeLeft(22)
//                .forward(27)
//                .strafeLeft(14)
//                .strafeTo(new Vector2d(2.5 * 24, -34))
//                .build();

        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(new Pose2d())
                        .strafeLeft(22)
                        .forward(27)
                        .strafeLeft(14)
                        .strafeTo(new Vector2d(2.5 * 24, -34))
                        .build()
        );

//        drive.followTrajectorySequence(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
    }

    public void SlideDropOff(SlideHeight height) throws InterruptedException {
        switch (height) {
            case High:
                slide.setTargetLevel(SlideHeight.High);
                break;

            case Medium:
                slide.setTargetLevel(SlideHeight.Medium);
                break;

            case Low:
                slide.setTargetLevel(SlideHeight.Low);
                break;
        }

        //Wait for the slide to reach position
        slide.waitFor();

        intake.Pickup();
        Thread.sleep(500);
        intake.Release();
    }


    public static double GridToInch(double Grids) {
        return Grids * 24;
    }

}
