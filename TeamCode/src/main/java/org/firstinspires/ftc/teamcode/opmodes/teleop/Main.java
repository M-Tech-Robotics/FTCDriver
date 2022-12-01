package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.systems.Controllers.linearSlide.linearSlide;
import org.firstinspires.ftc.teamcode.systems.Controllers.drivetrains.MainDriveTrain;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

@TeleOp(name = "Main Teleop", group = "Main")
public class Main extends LinearOpMode {
    MainDriveTrain Drive;
    linearSlide Slide;

    DcMotorEx IntakeMotor;

    GamepadEx Driver1;
    GamepadEx Driver2;

    double DriveSpeed = 1;
    double TurnSpeed = .6;
    boolean HasCone = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive = new MainDriveTrain(hardwareMap, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slide = new linearSlide(hardwareMap);

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
        }
    }

    public void SlideController() {
        Slide.leftSlide.setPower(-gamepad2.right_trigger);
        Slide.rightSlide.setPower(-gamepad2.right_trigger);


        if (gamepad2.dpad_down) {
            Slide.leftSlide.setPower(1.0);
            Slide.rightSlide.setPower(1.0);
        } else if (gamepad2.right_trigger == 0) {
            Slide.leftSlide.setPower(-0.40);
            Slide.rightSlide.setPower(-0.40);
        }

        telemetry.addData("Left Trigger", gamepad2.left_trigger);
        telemetry.addData("Right Trigger", gamepad2.right_trigger);
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
        if (gamepad2.left_bumper) {
            IntakeMotor.setPower(0.7);
        } else if (gamepad2.right_bumper) {
            IntakeMotor.setPower(-0.7);
        } else {
            IntakeMotor.setPower(0);
        }

        telemetry.addData("Left Bumper", gamepad2.left_bumper);
        telemetry.addData("Right Bumper", gamepad2.right_bumper);
        telemetry.update();

//        IntakeMotor.setPower(gamepad2.left_trigger);
//        IntakeMotor.setPower(-gamepad2.right_trigger);
    }

}