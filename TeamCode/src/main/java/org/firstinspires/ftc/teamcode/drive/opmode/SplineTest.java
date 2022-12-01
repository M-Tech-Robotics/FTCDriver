package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(24 * 1.5, 61.63, Math.toRadians(270)))
                .strafeTo(new Vector2d(1 * 24, 61.63))
                .strafeTo(new Vector2d(24 + 24, 61.63 - 27))
                .build();

        telemetry.addLine("test worked");

        drive.followTrajectorySequence(traj);
    }
}
