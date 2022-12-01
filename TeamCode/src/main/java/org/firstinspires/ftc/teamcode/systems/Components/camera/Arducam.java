package org.firstinspires.ftc.teamcode.systems.Components.camera;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.HardwareMap;
import android.annotation.SuppressLint;

import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Arducam {
    private WebcamName imuDevice;

    public Arducam(HardwareMap hardwareMap, String Name) {

    }

    public Arducam(HardwareMap hardwareMap) {
        this(hardwareMap, "imu");
    }

//    public double[] getAngles() {
//        Orientation orientation = imuDevice.getAngularOrientation();
//
//        return new double[]{orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle};
//    }
//
//    public double getAngle() {
//        double returnVal;
//        if (imuDevice.getAngularOrientation().firstAngle < 0) {
//            returnVal = Math.abs(imuDevice.getAngularOrientation().firstAngle);
//        } else {
//            returnVal = Math.abs(imuDevice.getAngularOrientation().firstAngle - 360);
//        }
//        return returnVal % 360;
//    }
//
//    public BNO055IMU getImu() {
//        return imuDevice;
//    }


}
