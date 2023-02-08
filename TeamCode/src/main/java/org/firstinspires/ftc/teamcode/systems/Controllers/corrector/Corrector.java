package org.firstinspires.ftc.teamcode.systems.Controllers.corrector;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class Corrector {
    public ServoEx CorrectorDevice;

    Positions currentState;
    public enum Positions {
        Out(90),
        In(0);
        public final int pos;

        Positions(int pos) {
            this.pos = pos;
        }
    }

    public Corrector(HardwareMap hardwareMap){
        CorrectorDevice = new SimpleServo(hardwareMap, "Corrector", 0, 120);

        CorrectorDevice.setPosition(Positions.In.pos);
    }

    public double getPosition() {
        return CorrectorDevice.getPosition();
    }

    public void goToAngle(double angle) {
        CorrectorDevice.turnToAngle(angle);
    }

    public void setState(Positions position) {
        currentState = position;
        CorrectorDevice.turnToAngle(position.pos);
    }

    public void setPosition(double position) {
        CorrectorDevice.setPosition(position);
    }


    public void In() {
        setState(Positions.In);
    }


    public void Out() {
        setState(Positions.Out);
    }
}
