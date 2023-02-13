package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Random;

public class MeepMeepTesting {
    private static double GridSize;
    static MeepMeep meepMeep;

    public static void main(String[] args) {
        meepMeep = new MeepMeep(800);
        GridSize = 70 / 3;

        meepMeep.getWindowFrame().setTitle("MeepMeep | FTC Debugger | ONLY FOR MICK");

//        Trajectory traj = trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(100, 100), 0)
//                .build();

        Vector2d POS = getRandomPosition();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(30, 40, Math.toRadians(360), Math.toRadians(360), 14.5)
                .setDriveTrainType(DriveTrainType.MECANUM)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(36, 61, toRadians(270)))
                                        .forward(48)
                                        .lineToSplineHeading(new Pose2d(33, 0, toRadians(180)))

                                        .waitSeconds(2)

                                        // Go To Stack
                                        .lineToSplineHeading(new Pose2d(33, 12, toRadians(0)))
                                        .lineToSplineHeading(new Pose2d(63, 12, toRadians(0)))

                                        .waitSeconds(2)

                                        // Go to Pole
                                        .lineToSplineHeading(new Pose2d(23.5, 13, toRadians(270)))
                                        .forward(4)
                                        .back(4)


                                        .build()
                );

//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(36, 61, toRadians(270)))
//
//                                .splineToLinearHeading(new Pose2d(34, 55.5, toRadians(180)), toRadians(0))
//                                .lineToSplineHeading(new Pose2d(33, 0, toRadians(180)))
//
//                                .waitSeconds(2)
//
//                                .lineToSplineHeading(new Pose2d(33, 12, toRadians(0)))
//                                .lineToSplineHeading(new Pose2d(63, 12, toRadians(0)))
//
//                                .waitSeconds(2)
//
//                                .lineToSplineHeading(new Pose2d(23.5, 13, toRadians(270)))
//                                .forward(4)
//                                .back(4)
//
//
//                                .build()
//                );

        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(30, 40, Math.toRadians(360), Math.toRadians(360), 14.5)
                .setDriveTrainType(DriveTrainType.MECANUM)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(GridToInch(1.5), -61.63, Math.toRadians(90)))
                                .strafeLeft(22)
                                .forward(GridToInch(1) + 3)
                                .strafeLeft(14)

                                .waitSeconds(2)

                                .strafeTo(new Vector2d(POS.getX(), -34))

                                .waitSeconds(5)
                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
//                .addEntity(mySecondBot)
                .start();
    }


    public static double GridToInch(double Grids) {
        return Grids * 24;
    }

    public static Vector2d getRandomPosition() {
        Random rand = new Random();
        int RandomNum = rand.nextInt(3);
        System.out.println(RandomNum);

        switch (RandomNum) {
            case(1):
                return new Vector2d(GridToInch(.5), 34);

            case(2):
                return new Vector2d(GridToInch(1.5), 34);

            default:
                return new Vector2d(GridToInch(2.5), 34);
        }
    }

}