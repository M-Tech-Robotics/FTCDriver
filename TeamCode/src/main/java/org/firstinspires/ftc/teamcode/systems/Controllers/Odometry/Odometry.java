package org.firstinspires.ftc.teamcode.systems.Controllers.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Odometry {
    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor horizontalEncoder;
    private BNO055IMU imu;

    private final double ticksPerInches = 8192;
    private final double robotEncoderWheelDistance = 7.125;
    private final double horizontalEncoderTickPerDegreeOffset = 4.001;


    private final Lock lock = new ReentrantLock();

    private volatile double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;

    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

    private String[] EncoderNames = {
            "IntakeMotor",
            "leftSlide",
            "rightSlide"
    };


    public Odometry(HardwareMap hardwareMap) {
        leftEncoder = hardwareMap.get(DcMotor.class, "IntakeMotor");
        rightEncoder = hardwareMap.get(DcMotor.class, "leftSlide");
        horizontalEncoder = hardwareMap.get(DcMotor.class, "rightSlide");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    public int[] returnRaw() {
        int[] positions = new int[3];
        positions[0] = rightEncoder.getCurrentPosition();
        positions[1] = leftEncoder.getCurrentPosition();
        positions[2] = horizontalEncoder.getCurrentPosition();
        return positions;
    }

    private double getImuAngle() {
        double returnVal;
        if (imu.getAngularOrientation().firstAngle < 0) {
            returnVal = Math.abs(imu.getAngularOrientation().firstAngle);
        } else {
            returnVal = Math.abs(imu.getAngularOrientation().firstAngle - 360);
        }
        return returnVal % 360;

    }


}
