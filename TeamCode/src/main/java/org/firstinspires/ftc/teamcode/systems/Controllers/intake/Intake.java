package org.firstinspires.ftc.teamcode.systems.Controllers.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.systems.Controllers.corrector.Corrector;

public class Intake {
    public DcMotorEx IntakeMotor;
    public Corrector corrector;

    public Intake(HardwareMap hardwareMap){
        corrector = new Corrector(hardwareMap);

        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void Pickup() throws InterruptedException {
        IntakeMotor.setPower(0.7);
        Thread.sleep(500);
        IntakeMotor.setPower(0);
    }

    public void Pickup(double time) throws InterruptedException {
        time *= 1000;

        IntakeMotor.setPower(0.7);
        Thread.sleep((long) time);
        IntakeMotor.setPower(0);
    }


    public void Release() throws InterruptedException {
        runThread(() -> {
            corrector.In();
        });
        IntakeMotor.setPower(-0.7);
        Thread.sleep(500);
        IntakeMotor.setPower(0);
    }

    void runThread(Runnable runnable) {
        Thread thread = new Thread(runnable);
        thread.start();
    }
}