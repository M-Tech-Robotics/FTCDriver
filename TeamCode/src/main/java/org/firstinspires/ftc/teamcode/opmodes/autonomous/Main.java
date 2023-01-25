package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Components.IMU;
import org.firstinspires.ftc.teamcode.systems.Controllers.intake.Intake;
import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Config
@Autonomous(name = "Main Auto", group = "Testing")
public class Main extends LinearOpMode {
    SampleMecanumDrive drive;
    LinearSlide slide;
    Thread e;
    Intake intake;
    DcMotorEx SlideEncoder;

    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;

    IMU IMU;

//    public TrajectorySequence test = drive.trajectorySequenceBuilder(new Pose2d())
//            .forward(48)
//            .lineToSplineHeading(new Pose2d(33, 0, toRadians(180)))
//
//            .waitSeconds(2)
//
//            // Go To Stack
//            .lineToSplineHeading(new Pose2d(33, 12, toRadians(0)))
//            .lineToSplineHeading(new Pose2d(63, 12, toRadians(0)))
//
//            .waitSeconds(2)
//
//            // Go to Pole
//            .lineToSplineHeading(new Pose2d(23.5, 13, toRadians(270)))
//            .forward(4)
//            .back(4)
//
//
//            .build();

//    TrajectorySequence dropPreload = drive.trajectorySequenceBuilder(new Pose2d(36, 61, toRadians(270)))
//            .forward(48)
//            .lineToSplineHeading(new Pose2d(33, 0, toRadians(180)))
//            .build();
//
//    TrajectorySequence goToStack = drive.trajectorySequenceBuilder(dropPreload.end())
//            .lineToSplineHeading(new Pose2d(33, 12, toRadians(0)))
//            .lineToSplineHeading(new Pose2d(63, 12, toRadians(0)))
//
//            .waitSeconds(2)
//
//            .build();
//
//    TrajectorySequence goToPole = drive.trajectorySequenceBuilder(goToStack.end())
//            .lineToSplineHeading(new Pose2d(23.5, 13, toRadians(270)))
//            .forward(4)
//            .back(4)
//            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        InitAll();

        TrajectorySequence test1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, toRadians(0)))
                .forward(48)
                .lineToSplineHeading(new Pose2d(62, -2, toRadians(-92))) //-6

//                .waitSeconds(2)

                // Go To Stack
//                .lineToSplineHeading(new Pose2d(48, 0, toRadians(85)))
//                .lineToSplineHeading(new Pose2d(54, 12, toRadians(0)))
//
//                .waitSeconds(2)
//
//                // Go to Pole
//                .lineToSplineHeading(new Pose2d(23.5, 13, toRadians(270)))
//                .forward(4)
//                .back(4)
                .build();
        Trajectory test2 = drive.trajectoryBuilder(test1.end())
                .forward(6)
                .build();
        //// //// ////

        TrajectorySequence test3 = drive.trajectorySequenceBuilder(test2.end())
                .back(6)
                .lineToSplineHeading(new Pose2d(50, 0, toRadians(0)))
//                .forward(34)
                .build();

        TrajectorySequence test4 = drive.trajectorySequenceBuilder(test3.end())
                .lineToSplineHeading(new Pose2d(50, -12, toRadians(0)))
                .build();



        waitForStart();


        if (isStopRequested()) return;

        new Thread(() -> {
           while (opModeIsActive() && !isStopRequested()) {
               Pose2d poseEstimate = drive.getPoseEstimate();
               telemetry.addData("Angle", IMU.getAngle());
               telemetry.addData("finalX", poseEstimate.getX());
               telemetry.addData("finalY", poseEstimate.getY());
               telemetry.addData("finalHeading", poseEstimate.getHeading());
               telemetry.update();
           }
        });

        drive.followTrajectorySequence(test1);

        slide.goTo(LinearSlide.Levels.High);

        drive.followTrajectory(test2);

        intake.Release();

        slide.goTo(LinearSlide.Levels.Low);

        drive.followTrajectorySequence(test3);


//        intake.Release();






//        slide.setTargetLevel(LinearSlide.Levels.Ground);

        telemetry.addData("Angle", IMU.getAngle());
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
    }

    void InitAll() {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        IMU = new IMU(hardwareMap);
        slide = new LinearSlide(hardwareMap);
    }

    void pickupCone() throws InterruptedException {
        slide.goTo(LinearSlide.Levels.Ground);
        intake.Pickup();

    }

    void dropCone() {

    }


    public static double GridToInch(double Grids) {
        return Grids * 24;
    }

}
