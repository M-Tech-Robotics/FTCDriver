package org.firstinspires.ftc.teamcode.systems.Controllers.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotorEx IntakeMotor;


    public Intake(HardwareMap hardwareMap){
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void Pickup() throws InterruptedException {
        IntakeMotor.setPower(0.7);
        Thread.sleep(500);
        IntakeMotor.setPower(0);
    }


    public void Release() throws InterruptedException {
        IntakeMotor.setPower(-0.7);
        Thread.sleep(500);
        IntakeMotor.setPower(0);
    }
}