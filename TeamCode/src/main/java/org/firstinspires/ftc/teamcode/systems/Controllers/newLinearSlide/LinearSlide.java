package org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;


/**
 * Linear Slide Controller
 * A class to control linear slides by controlling the positions
 */

public class LinearSlide {
    // Motor Objects //
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;

    // Slide States //
    public boolean autoMode;
    public boolean resetSlide = false;

    // Slide Data //
    private SlideHeight currentLevel = SlideHeight.Floor;

    /**
     * Linear Slide Constructor
     *
     * @param hardwareMap The hardware map from the OpMode
     */

    public LinearSlide(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setTargetPosition(SlideHeight.getEncoderCountFromEnum(SlideHeight.Floor));
        rightSlide.setTargetPosition(SlideHeight.getEncoderCountFromEnum(SlideHeight.Floor));


        leftSlide.setPower(1);
        rightSlide.setPower(1);


        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        autoMode = true;
    }

    public void waitFor() throws InterruptedException {
        do {
            Thread.sleep(40);
        } while (!this.isAtPosition(15));
    }


    public void setTargetPosition(int targetPosition) {
        leftSlide.setTargetPosition(targetPosition);
        rightSlide.setTargetPosition(targetPosition);
    }


    public void setTargetLevel(SlideHeight level) {
        leftSlide.setTargetPosition(SlideHeight.getEncoderCountFromEnum(level));
        rightSlide.setTargetPosition(SlideHeight.getEncoderCountFromEnum(level));
    }

    public boolean isAtPosition() {
        return !(rightSlide.isBusy());
    }

    public int getEncoderCount() {
        return rightSlide.getCurrentPosition();
    }

    public boolean isAtPosition(int tolerance) {
        return Math.abs(rightSlide.getCurrentPosition() - rightSlide.getTargetPosition()) < tolerance;
    }

    public void resetEncoder() {
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setTargetPosition(SlideHeight.getEncoderCountFromEnum(SlideHeight.Floor));
        rightSlide.setTargetPosition(SlideHeight.getEncoderCountFromEnum(SlideHeight.Floor));

        leftSlide.setPower(1);
        rightSlide.setPower(1);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setMotorPower(double power) {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public int getTargetHeight() {
        return rightSlide.getTargetPosition();
    }

    public void autoMode() {
        leftSlide.setTargetPosition(leftSlide.getCurrentPosition());
        rightSlide.setTargetPosition(rightSlide.getCurrentPosition());

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(1);
        rightSlide.setPower(1);

        autoMode = true;
    }

}
