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
import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.SlideHeight;

import java.io.File;

@Autonomous(name = "SlideTest", group = "Testing")
public class SlideTest extends LinearOpMode {
    LinearSlide slide;
    boolean inAction = false;
    Intake intake;


    @Override
    public void runOpMode() throws InterruptedException {
        slide = new LinearSlide(hardwareMap);
        intake = new Intake(hardwareMap);

        slide.setMotorPower(.4);
        waitForStart();

        //1244
        if (opModeIsActive() && !isStopRequested()) {
            goToPos(1227);
            goToPos(896);
            goToPos(405);
            goToPos(0);
        }

//        while (opModeIsActive() && !isStopRequested()) {
//            if (gamepad1.a && !inAction) {
//                inAction = true;
//
//                goToPos(1000);
//
//                sleep(400);
//
//                intake.Release();
//
//                inAction = false;
//            } else if (gamepad1.b && !inAction) {
//                inAction = true;
//
//                goToPos(200);
//
//                sleep(400);
//
//                intake.Pickup();
//
//                inAction = false;
//            }
//        }

    }


    public void goToPos(int Position) throws InterruptedException {
        slide.setTargetPosition(Position);

        do {
            Thread.sleep(40);
            slideTelemetry();
        } while (!slide.isAtPosition(8));

        sleep(2000);
    }
    public void slideTelemetry() {
        telemetry.addData("Position:", slide.getEncoderCount());

        telemetry.addData("Target:", slide.getTargetHeight());

        telemetry.addData("Distance:", Math.abs(slide.getEncoderCount() - slide.getTargetHeight()));

        telemetry.addData("State:", slide.isAtPosition());

        telemetry.update();
    }
}
