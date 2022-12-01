package org.firstinspires.ftc.teamcode.systems.Components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class IMU {
    private BNO055IMU imuDevice;

    public IMU(HardwareMap hardwareMap, String Name) {
        imuDevice = hardwareMap.get(BNO055IMU.class, Name);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imuDevice.initialize(parameters);
    }

    public IMU(HardwareMap hardwareMap) {
        this(hardwareMap, "imu");
    }

    public double[] getAngles() {
        Orientation orientation = imuDevice.getAngularOrientation();

        return new double[]{orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle};
    }

    public double getAngle() {
        double returnVal;
        if (imuDevice.getAngularOrientation().firstAngle < 0) {
            returnVal = Math.abs(imuDevice.getAngularOrientation().firstAngle);
        } else {
            returnVal = Math.abs(imuDevice.getAngularOrientation().firstAngle - 360);
        }
        return returnVal % 360;
    }

    public BNO055IMU getImu() {
        return imuDevice;
    }


}
