package org.firstinspires.ftc.teamcode.systems.Controllers.vision.AprilTag;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AprilTag {
    OpenCvCamera camera;
    AprilTagPipeline aprilTagPipeline;

    private Telemetry telemetry;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    public enum Tags {
        Left(16),
        Middle(18),
        Right(19);

        public final int tag;

        Tags(int tag) {
            this.tag = tag;
        }
    }

    int[] FTCTags = {
            16,
            18,
            19
    };

    // UNITS ARE METERS
    double tagsize = 0.166;

    public AprilTagDetection tagOfInterest = null;
    public Tags currentTag;

    public AprilTag(HardwareMap hardwareMap, Telemetry tele) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagPipeline = new AprilTagPipeline(tagsize, fx, fy, cx, cy);
        telemetry = tele;

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
    }

    public Tags DetectTags() throws InterruptedException {
        ArrayList<AprilTagDetection> currentDetections = aprilTagPipeline.getLatestDetections();
        boolean tagFound = false;

        if(currentDetections.size() != 0)
        {
            for(AprilTagDetection tag : currentDetections) {
                for (Tags teamTag : Tags.values()) {
                    if(tag.id == teamTag.tag) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

            }


            if(tagFound) {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        } else {
            telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null) {
                telemetry.addLine("(The tag has never been seen)");
            } else {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }



        telemetry.update();

        if (tagFound) {
            return findTag(tagOfInterest);
        } else {
            return null;
        }
    }

    public Tags getTag() throws InterruptedException {
        do {
            currentTag = DetectTags();
            Thread.sleep(20);
        } while (currentTag == null);

        return currentTag;
    }

    public static Tags findTag(AprilTagDetection id) {
        for(Tags tag : Tags.values()) {
            if (tag.tag == id.id) {
                return tag;
            }
        }
        return null;
    }

    public Tags findTag() {
        for(Tags tag : Tags.values()) {
            if (tag.tag == tagOfInterest.id) {
                return tag;
            }
        }
        return null;
    }
    public String getCurrentPark() {
        Tags tag = findTag();

        return tag.name();
    }

    public void logDetection() {
        tagToTelemetry(tagOfInterest);
    }
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected Program=%s", findTag(detection).name()));
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}
