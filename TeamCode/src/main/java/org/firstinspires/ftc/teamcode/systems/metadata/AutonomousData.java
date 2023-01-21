package org.firstinspires.ftc.teamcode.systems.metadata;

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


public class AutonomousData {
    SampleMecanumDrive drive;

    public class POS {
        public TrajectorySequence dropPreload = drive.trajectorySequenceBuilder(new Pose2d(36, 61, toRadians(270)))
                .forward(48)
                .lineToSplineHeading(new Pose2d(33, 0, toRadians(180)))
                .build();

        public TrajectorySequence goToStack = drive.trajectorySequenceBuilder(dropPreload.end())
                .lineToSplineHeading(new Pose2d(33, 12, toRadians(0)))
                .lineToSplineHeading(new Pose2d(63, 12, toRadians(0)))
                .waitSeconds(2)
                .build();

        public TrajectorySequence goToPole = drive.trajectorySequenceBuilder(goToStack.end())
                .lineToSplineHeading(new Pose2d(23.5, 13, toRadians(270)))
                .forward(4)
                .back(4)
                .build();
    }


}
