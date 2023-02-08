package org.firstinspires.ftc.teamcode.opmodes.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.systems.Controllers.corrector.Corrector;
import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.LinearSlide;

@TeleOp(name = "correctorTest", group = "Testing")
public class correctorTest extends LinearOpMode {
    public Corrector corrector;
    public int highest = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        corrector = new Corrector(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            corrector.CorrectorDevice.rotateByAngle(gamepad1.left_stick_y * 0.5);

            if (gamepad1.a) {
                corrector.setState(Corrector.Positions.In);
            }

            if (gamepad1.b) {
                corrector.setState(Corrector.Positions.Out);
            }

            mainTelemetry();
//            SlideController();
        }
    }


    public void SlideController() {
//        leftSlide.setPower(gamepad1.right_trigger/2);
//        rightSlide.setPower(gamepad1.right_trigger/2);
//
//        if (gamepad1.b) {
//            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//
//        if (gamepad1.a) {
//            highest = 0;
//        }
//
//        if (gamepad1.dpad_up) {
//            leftSlide.setPower(0.40);
//            rightSlide.setPower(0.40);
//        } else if (gamepad1.dpad_down) {
//            leftSlide.setPower(-0.40);
//            rightSlide.setPower(-0.40);
//        }

        mainTelemetry();
    }

    void mainTelemetry() {
        telemetry.addData("Servo Position:", corrector.CorrectorDevice.getPosition());
        telemetry.addData("Servo Angle:", corrector.CorrectorDevice.getAngle());
        telemetry.addData("Gamepad Y:", gamepad1.left_stick_y * .1);
        telemetry.update();
    }

}
