package org.firstinspires.ftc.teamcode.systems.Components.sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSense {
    public DistanceSensor FrontLeft;
    public DistanceSensor FrontRight;
    public DistanceSensor left;
    public DistanceSensor right;

    public DistanceUnit unit = DistanceUnit.INCH;

    public enum Side {
        Left, Right, None
    }


    public DistanceSense(HardwareMap hardwareMap) {
        FrontLeft = hardwareMap.get(DistanceSensor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DistanceSensor.class, "FrontRight");
        left = hardwareMap.get(DistanceSensor.class, "Left");
        right = hardwareMap.get(DistanceSensor.class, "Right");
    }

}
