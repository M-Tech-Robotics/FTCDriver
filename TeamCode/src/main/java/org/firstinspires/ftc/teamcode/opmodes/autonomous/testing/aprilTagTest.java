package org.firstinspires.ftc.teamcode.opmodes.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Controllers.corrector.Corrector;
import org.firstinspires.ftc.teamcode.systems.Controllers.vision.AprilTag.AprilTag;
import org.openftc.apriltag.AprilTagDetection;

@TeleOp(name = "aprilTagTest", group = "Testing")
public class aprilTagTest extends LinearOpMode {
    public AprilTag aprilTag;
    static final double FEET_PER_METER = 3.28084;
    public AprilTag.Tags tag;

    @Override
    public void runOpMode() throws InterruptedException {
        aprilTag = new AprilTag(hardwareMap, telemetry);

        detect();

        aprilTag.logDetection();
    }


    void detect() throws InterruptedException {
        while (!isStarted() && !isStopRequested()) {
            if (tag == null) {
                tag = aprilTag.getTag();
            }
        }
    }



    public void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected Program=%s", AprilTag.findTag(detection).name()));
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}
