/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.systems.Controllers.vision.AprilTag;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Components.IMU;
import org.firstinspires.ftc.teamcode.systems.Controllers.drivetrains.MainDriveTrain;
import org.firstinspires.ftc.teamcode.systems.Controllers.intake.Intake;
import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Config
@Autonomous(name = "Main Vision", group = "Testing")
public class AprilTagVision extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagPipeline aprilTagPipeline;
    IMU IMU;

    SampleMecanumDrive drive;
    LinearSlide slide;
    Thread e;
    Intake intake;

    public boolean debug = true;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;


    int[] FTCTags = {
            16,
            18,
            19
    };

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        InitAll();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagPipeline = new AprilTagPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */




        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    for (int TeamTag : FTCTags) {
                        if(tag.id == TeamTag)
                        {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }

                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
        }
        else
        {
            TrajectorySequence test1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, toRadians(0)))
//                .forward(48)
                    .lineToSplineHeading(new Pose2d(63, -2, toRadians(-90))) //-6
                    .build();
            TrajectorySequence test2 = drive.trajectorySequenceBuilder(test1.end())
                    .forward(6)
                    .build();

            TrajectorySequence test3 = drive.trajectorySequenceBuilder(test2.end())
                    .back(6)
                    .lineToSplineHeading(new Pose2d(53, 0, toRadians(90)))
                    .forward(30)
                    .build();

            TrajectorySequence test4 = drive.trajectorySequenceBuilder(test3.end())
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

            TrajectorySequence startPark = drive.trajectorySequenceBuilder(test4.end())
                    .lineToSplineHeading(new Pose2d(52, 0, toRadians(0)))
                    .build();

            TrajectorySequence park = tagToPath(tagOfInterest.id, startPark.end());
            ///// ///// /////

            runThread(() -> {
                try {
                    intake.Pickup();
                    slide.goTo(LinearSlide.Levels.High);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

            });



            drive.followTrajectorySequence(test1);

            runThread(() -> {
                try {
                    slide.goTo(LinearSlide.Levels.High);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

            });


            drive.followTrajectorySequence(test2);

            runThread(() -> {
                try {
                    intake.Release();
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

            });


            drive.followTrajectorySequence(test3);

            slide.setTargetPosition(218);
            Thread.sleep(600);
            intake.Pickup();
            slide.goTo(LinearSlide.Levels.High);

            drive.followTrajectorySequence(test4);



            drive.followTrajectorySequence(startPark);

            drive.followTrajectorySequence(park);

            runThread(() -> {
                try {
                    slide.goTo(LinearSlide.Levels.Ground);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

            });

            telemetry.addLine(String.valueOf(tagOfInterest.id));
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


    void runThread(Runnable runnable) {
        Thread thread = new Thread(runnable);
        thread.start();
    }
    TrajectorySequence tagToPath(int tag, Pose2d pos) {
        switch (tag) {
            case (16):
                return drive.trajectorySequenceBuilder(pos)
                        .strafeLeft(23)
                        .build();
            case (18):
                return drive.trajectorySequenceBuilder(pos)
//                        .lineToSplineHeading(new Pose2d(50, 0, toRadians(0)))
                        .build();
            case (19):
                return drive.trajectorySequenceBuilder(pos)
                        .strafeRight(22)
                        .build();
        }
        return null;
    }


    void InitAll() {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        IMU = new IMU(hardwareMap);
        slide = new LinearSlide(hardwareMap);
    }
}