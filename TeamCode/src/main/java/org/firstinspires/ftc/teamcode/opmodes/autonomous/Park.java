package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.systems.Controllers.vision.AprilTag.AprilTag;
import org.firstinspires.ftc.teamcode.systems.Controllers.vision.AprilTag.CameraOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Park", group = "Testing")
public class Park extends CameraOpMode {
    public AprilTag aprilTag;
    static final double FEET_PER_METER = 3.28084;
    public AprilTag.Tags tag;

    @Override
    public void opModeMain() throws InterruptedException {
//        waitForStart();

        Trajectory goToPos = drive.trajectoryBuilder(new Pose2d(0, 0, toRadians(0)))
                .lineToSplineHeading(new Pose2d(51, 0, toRadians(0)))
                .build();

        Trajectory traj = tagToPath(goToPos.end());

        // // // // // Main Methods // // // // //


//        runThread(() -> {
//            while (opModeIsActive() && !isStopRequested()) {
//                telemetry.addData("Tag Name", CurrentTag.name());
//                telemetry.addData("Tag ID", CurrentTag.tag);
//                telemetry.update();
//            }
//        });

        // Prepare to drop cones in High Junction //


        // Start to Pole Trajectory //
        drive.followTrajectory(goToPos);

        // Readjust cone //


        // Pick up Cone //

        // Back to the stack //
        drive.followTrajectory(traj);

        intake.corrector.In();

        slide.goTo(LinearSlide.Levels.Ground);
//

//        corrector.In();

        debugTelementry();

    }

    public Trajectory tagToPath(Pose2d pos) {
        switch (CurrentTag.name()) {
            case("Left"):
                return drive.trajectoryBuilder(pos)
                        .strafeLeft(25)
                        .build();

            case("Middle"):
                return drive.trajectoryBuilder(pos)
                        .back(1)
                        .build();

            case("Right"):
                return drive.trajectoryBuilder(pos)
                        .strafeRight(25)
                        .build();
        }

        return null;
    }

    void debugTelementry() {
        telemetry.addData("Park Pos", CurrentTag.name());
        telemetry.update();
    }

    void pickupCones(int pos) throws InterruptedException {
        intake.corrector.In();
        slide.goTo(pos);
        intake.Pickup();
        slide.goTo(LinearSlide.Levels.High);
//        corrector.Out();
    }

    void runThread(Runnable runnable) {
        Thread thread = new Thread(runnable);
        thread.start();
    }
}
