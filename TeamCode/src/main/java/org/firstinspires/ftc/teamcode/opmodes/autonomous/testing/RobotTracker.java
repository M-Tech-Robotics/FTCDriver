package org.firstinspires.ftc.teamcode.opmodes.autonomous.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name = "RobotTracker", group = "Testing")
public class RobotTracker extends LinearOpMode {
    public static final double TRACKWIDTH = 9.915;
    public static final double CENTER_WHEEL_OFFSET = -2.1;
    public static final double WHEEL_DIAMETER = 1.88976;
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private MecanumDrive driveTrain;
    private Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;
    boolean controlMode = false;
    private FtcDashboard dashboard;


    @Override
    public void runOpMode() throws InterruptedException {
        leftOdometer = frontLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = frontRight.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = backLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();


        GamepadEx driverOp = new GamepadEx(gamepad1);


        waitForStart();

        while (!isStopRequested()) {
            if (controlMode) {
                driveTrain.driveFieldCentric(
                        driverOp.getLeftX(),
                        driverOp.getLeftY(),
                        driverOp.getRightX(),
                        imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                        false
                );
            }

            if (gamepad1.a) {
                controlMode = !controlMode;
            }


            odometry.updatePose(); // update the position

            Pose2d poseEstimate = odometry.getPose();

//            drawInterface();
            telemetry.addData("controlMode", controlMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    void InitAll() {
        frontLeft = new MotorEx(hardwareMap, "leftFront");
        frontRight = new MotorEx(hardwareMap, "rightFront");
        backLeft = new MotorEx(hardwareMap, "leftRear");
        backRight = new MotorEx(hardwareMap, "rightRear");

        driveTrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // Here we set the distance per pulse of the odometers.
        // This is to keep the units consistent for the odometry.
        leftOdometer = frontLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = frontRight.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = backLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        rightOdometer.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

    }

//    public void drawInterface() {
//        TelemetryPacket packet = new TelemetryPacket();
//        Canvas fieldOverlay = packet.fieldOverlay();
//        Pose2d poseEstimate = odometry.getPose();
//
////        packet.put("x", poseEstimate.getX());
////        packet.put("y", poseEstimate.getY());
////        packet.put("heading (deg)", Math.toDegrees(poseEstimate.getHeading()));
//
//        fieldOverlay.setStroke("#3F51B5");
//
//        fieldOverlay.strokeCircle(poseEstimate.getX(), poseEstimate.getY(), 7.25);
//
//        dashboard.sendTelemetryPacket(packet);
//    }
}
