package org.firstinspires.ftc.teamcode.opmodes.autonomous.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.systems.Controllers.intake.Intake;
import org.firstinspires.ftc.teamcode.systems.Controllers.linearSlide.linearSlide;
import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.SlideHeight;

import java.io.File;

@Autonomous(name = "SlideTest", group = "Testing")
public class SlideTest extends LinearOpMode {
    DcMotorEx SlideEncoder;
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;
    Intake intake;

    linearSlide Slide;


    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);

        SlideEncoder = hardwareMap.get(DcMotorEx.class, "linearMotor");
        SlideEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SlideEncoder.setTargetPosition(1244);


        SlideEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
//        Slide = new linearSlide(hardwareMap);
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setTargetPosition(-1244);
        rightSlide.setTargetPosition(-1244);


        leftSlide.setPower(1);
        rightSlide.setPower(1);


        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();

        sleep(2000);

        leftSlide.setTargetPosition(-1200);
        rightSlide.setTargetPosition(-1200);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        sleep(800);

        if (!SlideEncoder.isBusy()) {
            intake.Release();
        }



        //1244
        while (opModeIsActive() && !isStopRequested()) {


//            SlideController();
            telemetry.addData("CurrentPOS", SlideEncoder.getCurrentPosition());
            telemetry.update();
        }

    }
    public void SlideController() {
        Slide.leftSlide.setPower(-gamepad1.right_trigger);
        Slide.rightSlide.setPower(-gamepad1.right_trigger);


        if (gamepad1.dpad_down) {
            Slide.leftSlide.setPower(1.0);
            Slide.rightSlide.setPower(1.0);
        } else if (gamepad1.right_trigger == 0) {
            Slide.leftSlide.setPower(-0.40);
            Slide.rightSlide.setPower(-0.40);
        }
    }
}
