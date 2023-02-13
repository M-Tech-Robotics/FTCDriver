package org.firstinspires.ftc.teamcode.opmodes.teleop.Misc;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.systems.Controllers.corrector.Corrector;
import org.firstinspires.ftc.teamcode.systems.Controllers.drivetrains.MainDriveTrain;
import org.firstinspires.ftc.teamcode.systems.Controllers.linearSlide.linearSlide;

@TeleOp(name = "Declan's Controls", group = "Testing")
public class OnePerson extends LinearOpMode {
    MainDriveTrain Drive;
    linearSlide Slide;

    DcMotorEx IntakeMotor;
    Corrector corrector;
    GamepadEx Driver1;
    GamepadEx Driver2;

    double DriveSpeed = 1;
    double TurnSpeed = .6;
    boolean HasCone = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive = new MainDriveTrain(hardwareMap, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slide = new linearSlide(hardwareMap);
        corrector = new Corrector(hardwareMap);

        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Driver1 = new GamepadEx(gamepad1);
        Driver2 = new GamepadEx(gamepad2);


        waitForStart();

        telemetry.addData("Made by:", "Mick");
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()){
            Driver1Controls();
            SlideController();
            PickupCone();
//            CorrectorControls();
        }
    }

    public void SlideController() {
        Slide.leftSlide.setPower(-gamepad1.right_trigger);
        Slide.rightSlide.setPower(-gamepad1.right_trigger);


        if (gamepad1.left_trigger != 0 || gamepad1.dpad_down) {
            Slide.leftSlide.setPower(1.0);
            Slide.rightSlide.setPower(1.0);
        } else if (gamepad1.right_trigger == 0) {
            Slide.leftSlide.setPower(-0.40);
            Slide.rightSlide.setPower(-0.40);
        }

        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.update();
    }

    public void Driver1Controls() {
        Drive.leftRear.setPower(DriveSpeed * ((Driver1.getLeftY() - Driver1.getLeftX()) + (Driver1.getRightX())));
        Drive.leftFront.setPower(DriveSpeed * ((Driver1.getLeftY() + Driver1.getLeftX()) + (Driver1.getRightX())));
        Drive.rightRear.setPower(DriveSpeed * ((Driver1.getLeftY() + Driver1.getLeftX()) - (Driver1.getRightX())));
        Drive.rightFront.setPower(DriveSpeed * ((Driver1.getLeftY() - Driver1.getLeftX()) - (Driver1.getRightX())));

        if (gamepad1.y) {
            DriveSpeed = 1;
         } else if (gamepad1.b) {
            DriveSpeed = 0.8;
        } else if (gamepad1.a) {
            DriveSpeed = 0.5;
        }

        telemetry.addData("Drive Speed", DriveSpeed);
        telemetry.update();
    }

    public void PickupCone() {
        if (gamepad1.left_bumper) {
            IntakeMotor.setPower(0.7);
        } else if (gamepad1.right_bumper) {
            IntakeMotor.setPower(-0.7);
        } else {
            IntakeMotor.setPower(0);
        }

        telemetry.addData("Left Bumper", gamepad1.left_bumper);
        telemetry.addData("Right Bumper", gamepad1.right_bumper);
        telemetry.update();

//        IntakeMotor.setPower(gamepad1.left_trigger);
//        IntakeMotor.setPower(-gamepad1.right_trigger);
    }

    public void CorrectorControls() {
        if (gamepad1.dpad_left) {
            corrector.setState(Corrector.Positions.In);
        }

        if (gamepad1.dpad_right) {
            corrector.setState(Corrector.Positions.Out);
        }
    }
}