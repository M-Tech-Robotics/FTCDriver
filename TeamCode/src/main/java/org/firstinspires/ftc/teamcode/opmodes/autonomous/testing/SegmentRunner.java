package org.firstinspires.ftc.teamcode.opmodes.autonomous.testing;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Components.IMU;
import org.firstinspires.ftc.teamcode.systems.Controllers.corrector.Corrector;
import org.firstinspires.ftc.teamcode.systems.Controllers.intake.Intake;
import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Config
@Autonomous(name = "SegmentRunner", group = "Testing")
public class SegmentRunner extends LinearOpMode {
    SampleMecanumDrive drive;
    LinearSlide slide;
    Corrector corrector;
    Intake intake;
    DcMotorEx SlideEncoder;

    public String SegmentName = "GoToPole";

    IMU IMU;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        InitAll();

//        TrajectorySequence goToPole = drive.trajectorySequenceBuilder(new Pose2d(0, 0, toRadians(0)))


        if (isStopRequested()) return;

        TrajectorySequence currentSegment = runSegment(SegmentName, new Pose2d());

        runThread(() -> {
            try {
                intake.Pickup();
                slide.goTo(LinearSlide.Levels.High);
                corrector.Out();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        });


        drive.followTrajectorySequence(currentSegment);




        Thread.sleep(1500);
        debugTelementry();
    }

    void debugTelementry() {
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
        corrector = new Corrector(hardwareMap);
    }

    void pickupCone() throws InterruptedException {
        slide.goTo(LinearSlide.Levels.Ground);
        intake.Pickup();

    }


    TrajectorySequence runSegment(String name, Pose2d endPos) {
        switch (name) {
            case "StartToJunction":
                return drive.trajectorySequenceBuilder(new Pose2d(0, 0, toRadians(0)))
//                .forward(48)
                        .lineToSplineHeading(new Pose2d(61, -2, toRadians(-90)))
                        .build();


            case "MoveToStack":
                return drive.trajectorySequenceBuilder(endPos)
                        .back(6)
                        .lineToSplineHeading(new Pose2d(53, 0, toRadians(0)))
                        .turn(90)
                        .forward(28)
                        .build();
            case "GoToPole":
                return drive.trajectorySequenceBuilder(endPos)
                        .lineToSplineHeading(new Pose2d(50, -12, toRadians(0)))
                        .forward(8)
                        .waitSeconds(.5)
                        .addDisplacementMarker(() -> {
                            try {
                                intake.Release();
                            } catch (InterruptedException e) {
                                throw new RuntimeException(e);
                            }
                        })
                        .back(6)
                        .build();
        }

        return null;
    }

    void runThread(Runnable runnable) {
        Thread thread = new Thread(runnable);
        thread.start();
    }
    public static double GridToInch(double Grids) {
        return Grids * 24;
    }
}
