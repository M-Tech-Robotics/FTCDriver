package org.firstinspires.ftc.teamcode.systems.modules.templates;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.systems.Utils.MiscUtils;

public abstract class OpModeTemplate extends LinearOpMode {



    public abstract void opModeMain() throws InterruptedException;


    @Override
    public final void runOpMode() throws InterruptedException {

        try {
            opModeMain();
        } catch (InterruptedException e) {
            throw e;
        } catch (Exception e) {
            RobotLog.setGlobalErrorMsg(MiscUtils.getStackTraceAsString(e)); //Appends more information to the error message
        }
    }


    protected void checkStop() throws InterruptedException {
        if (this.isStopRequested() || Thread.currentThread().isInterrupted()) {
            throw new InterruptedException();
        }
    }
}
