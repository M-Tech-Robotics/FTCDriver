package org.firstinspires.ftc.teamcode.opmodes.autonomous.main;

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


@Config
@Autonomous(name = "Blue Autonomous", group = "Testing")
public class BlueAutonomous extends LinearOpMode {
    SampleMecanumDrive drive;
    Intake intake;
    DcMotorEx SlideEncoder;
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;
    IMU IMU;

    int HighJunct = 1200;
    int LowJunct = 100;
    int MidJunct = 444;
    int Ground = 0;

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
//        TrajectorySequence DropCone = drive.trajectorySequenceBuilder(new Pose2d(36, 61, toRadians(270)))
//                .forward(70)
//                .strafeRight(20)
//                .build();
//
//        Trajectory MoveToHighJunction = drive.trajectoryBuilder(DropCone.end())
//                .forward(9)
//                .build();



        waitForStart();


        if (isStopRequested()) return;

//        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
//                .strafeLeft(22)
//                .forward(27)
//                .strafeLeft(14)
//                .strafeTo(new Vector2d(2.5 * 24, -34))
//                .build();

//        drive.followTrajectorySequence(DropCone);
        LiftCone(HighJunct, 0.5, "High");
//        drive.followTrajectory(MoveToHighJunction);
//        DropCone();
        sleep(2000);
        LiftCone(LowJunct, 0.5, "Low");
        sleep(2000);

        LiftCone(MidJunct, 0.5, "Mid");
        sleep(2000);

        LiftCone(Ground, 0.5, "Ground");
        sleep(2000);


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

        leftSlide.setTargetPosition(SlideEncoder.getCurrentPosition());
        rightSlide.setTargetPosition(SlideEncoder.getCurrentPosition());

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void LiftCone(int Position, double Power, String debugName) throws InterruptedException {
        leftSlide.setTargetPosition(Position);
        SlideEncoder.setTargetPosition(Position);

        leftSlide.setPower(Power);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        do {
            Thread.sleep(40);
            telemetry.addData("Name", debugName);
            telemetry.addData("TargetPosition", SlideEncoder.getTargetPosition());
            telemetry.addData("CurrentPosition", SlideEncoder.getCurrentPosition());
            telemetry.addData("IsAtPos", Math.abs(SlideEncoder.getTargetPosition() - SlideEncoder.getCurrentPosition()));
            telemetry.update();
        } while (!getTols(20));
    }

    public void DropCone() {
        leftSlide.setTargetPosition(-200);
        rightSlide.setTargetPosition(-200);
    }

    public boolean getTols(int Tol) {
        return Math.abs(SlideEncoder.getTargetPosition() - SlideEncoder.getCurrentPosition()) < Tol;
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

    public int currentPos() {
        return Math.abs(SlideEncoder.getCurrentPosition());
    }

    public void waitFor() throws InterruptedException {
        do {
            sleep(40);
            telemetry.addData("CurrentPos", Math.abs(SlideEncoder.getCurrentPosition() - SlideEncoder.getTargetPosition()));
            telemetry.update();
        } while (rightSlide.isBusy());
    }


    public static double GridToInch(double Grids) {
        return Grids * 24;
    }

}
