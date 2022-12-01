package org.firstinspires.ftc.teamcode.systems.Controllers.drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MainDriveTrain {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;


    public MainDriveTrain(HardwareMap hardwareMap, DcMotor.RunMode runMode){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setRunMode(runMode);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void setRunMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        leftRear.setMode(runMode);
        rightRear.setMode(runMode);
        rightFront.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior Behavior) {
        leftFront.setZeroPowerBehavior(Behavior);
        leftRear.setZeroPowerBehavior(Behavior);
        rightRear.setZeroPowerBehavior(Behavior);
        rightFront.setZeroPowerBehavior(Behavior);
    }
}