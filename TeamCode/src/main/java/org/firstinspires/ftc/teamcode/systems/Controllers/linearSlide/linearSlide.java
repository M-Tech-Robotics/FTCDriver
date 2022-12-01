package org.firstinspires.ftc.teamcode.systems.Controllers.linearSlide;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class linearSlide {
    public DcMotor leftSlide, rightSlide;



    public linearSlide(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);


    }


    public void setRunMode(DcMotor.RunMode runMode) {
        leftSlide.setMode(runMode);
        rightSlide.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior Behavior) {
        leftSlide.setZeroPowerBehavior(Behavior);
        rightSlide.setZeroPowerBehavior(Behavior);
    }
}
