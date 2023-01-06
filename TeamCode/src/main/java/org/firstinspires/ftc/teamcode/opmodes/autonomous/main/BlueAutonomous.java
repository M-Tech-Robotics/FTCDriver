package org.firstinspires.ftc.teamcode.opmodes.autonomous.main;

import static java.lang.Math.toRadians;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.teleop.testing.SlideTest;
import org.firstinspires.ftc.teamcode.systems.Components.IMU;
import org.firstinspires.ftc.teamcode.systems.Controllers.intake.Intake;
import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.SlideHeight;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Config
@Autonomous(name = "Blue Autonomous", group = "Testing")
public class BlueAutonomous extends LinearOpMode {
    SampleMecanumDrive drive;
    Intake intake;
    DcMotorEx SlideEncoder;
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;
    IMU IMU;

    TrajectorySequence leftParking = drive.trajectorySequenceBuilder(new Pose2d())
            .back(9)
            .strafeLeft(30)
            .build();
    TrajectorySequence centerParking = drive.trajectorySequenceBuilder(new Pose2d())
            .back(9)
            .strafeLeft(15)
            .build();
    TrajectorySequence rightParking = drive.trajectorySequenceBuilder(new Pose2d())
            .back(9)
            .strafeRight(10)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        InitAll();

        //// //// ////
        TrajectorySequence DropCone = drive.trajectorySequenceBuilder(new Pose2d(36, 61, toRadians(270)))
                .forward(70)
                .strafeRight(20)
                .build();

        Trajectory MoveToHighJunction = drive.trajectoryBuilder(DropCone.end())
                .forward(9)
                .build();



        waitForStart();


        if (isStopRequested()) return;

//        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
//                .strafeLeft(22)
//                .forward(27)
//                .strafeLeft(14)
//                .strafeTo(new Vector2d(2.5 * 24, -34))
//                .build();

        drive.followTrajectorySequence(DropCone);
        LiftCone();
        drive.followTrajectory(MoveToHighJunction);

        do {
            sleep(40);
        } while (leftSlide.isBusy());

//        waitFor();

        intake.Release();








        telemetry.addData("Angle", IMU.getAngle());

//        drive.followTrajectorySequence(trajectory);

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
        //// //// ////
        SlideEncoder = hardwareMap.get(DcMotorEx.class, "linearMotor");
        SlideEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //// //// ////
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void LiftCone() {
        leftSlide.setTargetPosition(-1244);
        rightSlide.setTargetPosition(-1244);


        leftSlide.setPower(.5);
        rightSlide.setPower(.5);


        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

//    public void SlideDropOff(SlideHeight height) throws InterruptedException {
//        switch (height) {
//            case High:
//                slide.setTargetLevel(SlideHeight.High);
//                break;
//
//            case Medium:
//                slide.setTargetLevel(SlideHeight.Medium);
//                break;
//
//            case Low:
//                slide.setTargetLevel(SlideHeight.Low);
//                break;
//        }
//
//        //Wait for the slide to reach position
//        slide.waitFor();
//
//        intake.Pickup();
//        Thread.sleep(500);
//        intake.Release();
//    }

    public boolean isAtPosition(int tolerance) {
        return Math.abs(SlideEncoder.getCurrentPosition() - 1244) < tolerance;
    }

    public void waitFor() throws InterruptedException {
        do {
            sleep(40);
            telemetry.addData("CurrentPos", Math.abs(SlideEncoder.getCurrentPosition() - SlideEncoder.getTargetPosition()));
            telemetry.addData("Is At POS", isAtPosition(15));
            telemetry.update();
        } while (rightSlide.isBusy());
    }


    public static double GridToInch(double Grids) {
        return Grids * 24;
    }

}
