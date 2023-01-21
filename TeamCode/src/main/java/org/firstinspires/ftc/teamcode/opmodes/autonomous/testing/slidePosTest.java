package org.firstinspires.ftc.teamcode.opmodes.autonomous.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.LinearSlide;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name = "slidePosTest", group = "Testing")
public class slidePosTest extends LinearOpMode {
    public LinearSlide slide;
    public DcMotorEx leftSlide, rightSlide;
    public int highest = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        InitSlides();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            SlideController();
        }
    }

    public void InitSlides() {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addLine("Linear Slide Initialized.");
        telemetry.update();
    }

    public void SlideController() {
        leftSlide.setPower(gamepad1.right_trigger/2);
        rightSlide.setPower(gamepad1.right_trigger/2);

        if (gamepad1.b) {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (gamepad1.a) {
            highest = 0;
        }

        if (gamepad1.dpad_up) {
            leftSlide.setPower(0.40);
            rightSlide.setPower(0.40);
        } else if (gamepad1.dpad_down) {
            leftSlide.setPower(-0.40);
            rightSlide.setPower(-0.40);
        }

        slideTelemetry();
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

    public int getTargetHeight() {
        return leftSlide.getTargetPosition();
    }

    void slideTelemetry() {
        telemetry.addData("Left Position:", leftSlide.getCurrentPosition());
        telemetry.addData("Right Position:", rightSlide.getCurrentPosition());

        telemetry.addData("Left Target:", leftSlide.getTargetPosition());
        telemetry.addData("Right Target:", rightSlide.getTargetPosition());

        telemetry.addData("Distance:", Math.abs(leftSlide.getCurrentPosition() - leftSlide.getTargetPosition()));

        telemetry.addData("Left State:", leftSlide.isBusy());
        telemetry.addData("Right State:", rightSlide.isBusy());

        if (leftSlide.getCurrentPosition() > highest) {
            highest = leftSlide.getCurrentPosition();
        }
        telemetry.addData("Highest Height:", highest);

        telemetry.update();
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
