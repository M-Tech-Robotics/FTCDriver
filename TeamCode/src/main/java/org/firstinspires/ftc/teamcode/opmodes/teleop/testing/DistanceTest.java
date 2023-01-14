package org.firstinspires.ftc.teamcode.opmodes.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class DistanceTest extends LinearOpMode {
    DistanceSensor FrontLeft;
    DistanceSensor FrontRight;
    DistanceSensor left;
    DistanceSensor right;

    DistanceUnit unit = DistanceUnit.INCH;

    @Override
    public void runOpMode() {
        FrontLeft = hardwareMap.get(DistanceSensor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DistanceSensor.class, "FrontRight");
        left = hardwareMap.get(DistanceSensor.class, "Left");
        right = hardwareMap.get(DistanceSensor.class, "Right");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("FrontLeft Distance", FrontLeft.getDistance(unit));
            telemetry.addData("FrontRight Distance", FrontRight.getDistance(unit));
            telemetry.addData("Left Distance", left.getDistance(unit));
            telemetry.addData("Right Distance", right.getDistance(unit));
            telemetry.update();
        }
    }
}