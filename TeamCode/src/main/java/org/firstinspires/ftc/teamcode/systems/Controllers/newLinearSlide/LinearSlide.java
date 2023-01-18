package org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Arrays;
import java.util.List;


/**
 * Linear Slide Controller
 * A class to control linear slides by controlling the positions
 */

public class LinearSlide {
    // Motor Objects //
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;

    private List<DcMotorEx> slides;


    public enum Levels {
        High(0),
        Mid(0),
        Low(0),
        Ground(0);

        public final int pos;

        Levels(int pos) {
            this.pos = pos;
        }
    }


    public LinearSlide(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

//        slides = Arrays.asList(leftSlide, rightSlide);
//
//        for (DcMotorEx slide : slides) {
//            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            slide.setTargetPosition(Levels.Ground.pos);
//            slide.setPower(1);
//            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }


        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftSlide.setTargetPosition(Levels.Ground.pos);
        rightSlide.setTargetPosition(Levels.Ground.pos);


        leftSlide.setPower(1);
        rightSlide.setPower(1);


        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void setTargetPosition(int targetPosition) {
        leftSlide.setTargetPosition(targetPosition);
        rightSlide.setTargetPosition(targetPosition);
    }

    public void setTargetLevel(Levels level) {
        leftSlide.setTargetPosition(level.pos);
        rightSlide.setTargetPosition(level.pos);
    }

    public boolean isAtPosition() {
        return !(leftSlide.isBusy());
    }

    public boolean isAtPosition(int tolerance) {
        return Math.abs(leftSlide.getCurrentPosition() - leftSlide.getTargetPosition()) < tolerance;
    }
    public int getEncoderCount() {
        return leftSlide.getCurrentPosition();
    }


    public void waitOn() throws InterruptedException {
        do {
            Thread.sleep(40);
        } while (!this.isAtPosition(15));
    }

    public void goTo(Levels level) throws InterruptedException {
        setTargetLevel(level);

        do {
            Thread.sleep(40);
        } while (!this.isAtPosition(15));
    }

    public void goTo(int Position) throws InterruptedException {
        setTargetPosition(Position);

        do {
            Thread.sleep(40);
        } while (!this.isAtPosition(15));
    }

    public void reset() {
//        List<DcMotorEx> slides = Arrays.asList(leftSlide, rightSlide);
//
//        for (DcMotorEx slide : slides) {
//            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            slide.setTargetPosition(Levels.Ground.pos);
//            slide.setPower(1);
//            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftSlide.setTargetPosition(Levels.Ground.pos);
        rightSlide.setTargetPosition(Levels.Ground.pos);


        leftSlide.setPower(1);
        rightSlide.setPower(1);


        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void autoMode() {
     List<DcMotorEx> slides = Arrays.asList(leftSlide, rightSlide);

        for (DcMotorEx slide : slides) {
            slide.setTargetPosition(slide.getCurrentPosition());
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
        }
    }

    public void setMotorPower(double power) {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }


    public int getTargetHeight() {
        return leftSlide.getTargetPosition();
    }

}
