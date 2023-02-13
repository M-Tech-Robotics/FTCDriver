package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.systems.Controllers.vision.AprilTag.AprilTag;
import org.firstinspires.ftc.teamcode.systems.Controllers.vision.AprilTag.CameraOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Testing Auto", group = "Testing")
public class Testing extends CameraOpMode {
    public AprilTag aprilTag;
    static final double FEET_PER_METER = 3.28084;
    public AprilTag.Tags tag;

    @Override
    public void opModeMain() throws InterruptedException {

        TrajectorySequence goToPole = drive.trajectorySequenceBuilder(new Pose2d(0, 0, toRadians(0)))
//                .forward(48)
                .lineToSplineHeading(new Pose2d(61, -2, toRadians(-90))) //-6
                .build();
        TrajectorySequence scootUp = drive.trajectorySequenceBuilder(goToPole.end())
                .forward(6)
                .build();

        TrajectorySequence backUp = drive.trajectorySequenceBuilder(scootUp.end())
                .back(6)
                .setTurnConstraint(325.473, toRadians(100))
                .lineToSplineHeading(new Pose2d(51, 0, toRadians(86)))
                .runThread(() -> {
                    intake.corrector.In();
                })
                .forward(30)
                .build();

        TrajectorySequence dropCones = drive.trajectorySequenceBuilder(backUp.end())
                .back(48)
                .runThread(() -> {
                    try {
                        intake.Pickup();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .lineToSplineHeading(new Pose2d(50, -11, toRadians(0)))
                .forward(7)
                .waitSeconds(.5)
                .addDisplacementMarker(() -> {
                    try {
                        intake.Release();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .lineToSplineHeading(new Pose2d(50, 0, toRadians(0)))
                .build();





        TrajectorySequence backToCones = drive.trajectorySequenceBuilder(dropCones.end())
                .lineToSplineHeading(new Pose2d(51, 0, toRadians(88)))
                .forward(30)
                .build();









        Trajectory traj = tagToPath(dropCones.end());

        // // // // // Main Methods // // // // //


//        runThread(() -> {
//            while (opModeIsActive() && !isStopRequested()) {
//                telemetry.addData("Tag Name", CurrentTag.name());
//                telemetry.addData("Tag ID", CurrentTag.tag);
//                telemetry.update();
//            }
//        });

        // Prepare to drop cones in High Junction //
        runThread(() -> {
            try {
                intake.Pickup();
                slide.goTo(LinearSlide.Levels.High);
//                corrector.Out();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        });

        // Start to Pole Trajectory //
        drive.followTrajectorySequence(goToPole);

        // Readjust cone //
        runThread(() -> {
            try {
                intake.Pickup();
                slide.goTo(LinearSlide.Levels.High);
//                corrector.Out();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        });

        // Align Robot with Junction //
        drive.followTrajectorySequence(scootUp);

        // Drop Cone //
        runThread(() -> {
            try {
                intake.Release();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

        });

        // Go to junction //
        drive.followTrajectorySequence(backUp);

        // Pick up Cone //
        pickupCones(218);
//

        runThread(() -> {
            try {
                intake.Pickup();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

        });
        // Drop cone on Junction //
        drive.followTrajectorySequence(dropCones);

        // Pick up Cone //

        drive.followTrajectorySequence(backUp);

        // Pick up Cone //
        pickupCones(190);
//

        runThread(() -> {
            try {
                intake.Pickup();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

        });
        // Drop cone on Junction //
        drive.followTrajectorySequence(dropCones);

        // Back to the stack //
        drive.followTrajectory(traj);

        intake.corrector.In();

        slide.goTo(LinearSlide.Levels.Ground);
    }

    public Trajectory tagToPath(Pose2d pos) {
        switch (CurrentTag.name()) {
            case("Left"):
                return drive.trajectoryBuilder(new Pose2d(51, 0, toRadians(0)))
                        .strafeLeft(25)
                        .build();

            case("Middle"):
                return drive.trajectoryBuilder(new Pose2d(51, 0, toRadians(0)))
                        .back(1)
                        .build();

            case("Right"):
                return drive.trajectoryBuilder(new Pose2d(51, 0, toRadians(0)))
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
