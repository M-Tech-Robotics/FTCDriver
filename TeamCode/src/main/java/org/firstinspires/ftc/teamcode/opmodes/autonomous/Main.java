package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

    Intake intake;
    DcMotorEx SlideEncoder;

    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;

    IMU IMU;

    TrajectorySequence dropPreload = drive.trajectorySequenceBuilder(new Pose2d(36, 61, toRadians(270)))
            .forward(48)
            .lineToSplineHeading(new Pose2d(33, 0, toRadians(180)))
            .build();

    TrajectorySequence goToStack = drive.trajectorySequenceBuilder(dropPreload.end())
            .lineToSplineHeading(new Pose2d(33, 12, toRadians(0)))
            .lineToSplineHeading(new Pose2d(63, 12, toRadians(0)))

            .waitSeconds(2)

            .build();

    TrajectorySequence goToPole = drive.trajectorySequenceBuilder(goToStack.end())
            .lineToSplineHeading(new Pose2d(23.5, 13, toRadians(270)))
            .forward(4)
            .back(4)
            .build();


//    TrajectorySequence leftParking = drive.trajectorySequenceBuilder(new Pose2d())
//            .back(9)
//            .strafeLeft(30)
//            .build();
//    TrajectorySequence centerParking = drive.trajectorySequenceBuilder(new Pose2d())
//            .back(9)
//            .strafeLeft(15)
//            .build();
//    TrajectorySequence rightParking = drive.trajectorySequenceBuilder(new Pose2d())
//            .back(9)
//            .strafeRight(10)
//            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        InitAll();

        //// //// ////




        waitForStart();


        if (isStopRequested()) return;
        drive.followTrajectorySequence(dropPreload);


//        intake.Release();

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
