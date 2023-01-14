package org.firstinspires.ftc.teamcode.opmodes.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.teleop.Main;
import org.firstinspires.ftc.teamcode.systems.Controllers.drivetrains.MainDriveTrain;

@Autonomous(name = "TrackingTest", group = "Testing")
public class TrackingTest extends LinearOpMode {
    public MainDriveTrain drive;




    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MainDriveTrain(hardwareMap, DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();



    }
}
