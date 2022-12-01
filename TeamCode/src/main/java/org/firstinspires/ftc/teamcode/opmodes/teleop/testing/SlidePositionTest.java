package org.firstinspires.ftc.teamcode.opmodes.teleop.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.SlideHeight;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ServiceLoader;

@Config
@TeleOp(name = "Slide Position Test", group = "Testing")
public class SlidePositionTest extends LinearOpMode{
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public LinearSlide Slide;
    public int POS = 0;
    public boolean wasReset = false;
    private final ElapsedTime slideResetTimer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        Slide = new LinearSlide(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Linear Slide Initialized.");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();


        while (opModeIsActive() && !isStopRequested()){
            int pos = (int) (Slide.getTargetHeight() + (10 * -gamepad1.left_stick_y));

            if (Slide.getTargetHeight() < 0) {
                Slide.setTargetPosition(0);
            }
            if (Slide.getTargetHeight() > 720) {
                Slide.setTargetPosition(720);
            }

            if (Slide.isAtPosition()) {
                if (gamepad1.dpad_up) {
                    Slide.setTargetPosition(720);
                    Slide.waitFor();
                }

                if (gamepad1.dpad_down) {
                    Slide.setTargetPosition(0);
                    Slide.waitFor();
                }

                if (gamepad1.dpad_left) {
                    Slide.setTargetPosition(150);
                    Slide.waitFor();
                }

                if (gamepad1.dpad_right) {
                    Slide.setTargetPosition(250);
                    Slide.waitFor();
                }
            }



            

            if (gamepad1.a) {
                POS = 0;
                wasReset = true;
            }

            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Stick Calculation", (10 * -gamepad1.left_stick_y));
            telemetry.addData("Raw POS", pos);
            telemetry.addData("Slide Height", Slide.getEncoderCount());
            telemetry.addData("Last Recorded Height", POS);
            telemetry.addData("Restarted", wasReset);

            if (wasReset) {
                wasReset = false;
            }

            telemetry.update();
        }

    }

    public void SlideControl() {
        if (Math.abs(gamepad1.left_stick_y) > 0.1) {
            int pos = (int) (Math.abs(Slide.getEncoderCount()) + (100 * -gamepad1.left_stick_y));

            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Stick Calculation", (100 * -gamepad1.left_stick_y));
            telemetry.addData("Raw POS", pos);
            telemetry.update();

            POS = pos;

            if (pos < -100) {
                pos = -100;
            }

            if (pos > SlideHeight.getEncoderCountFromEnum(SlideHeight.High)) {
                pos = SlideHeight.getEncoderCountFromEnum(SlideHeight.High);
            }

            Slide.setTargetPosition(pos);
        }
    }

}
