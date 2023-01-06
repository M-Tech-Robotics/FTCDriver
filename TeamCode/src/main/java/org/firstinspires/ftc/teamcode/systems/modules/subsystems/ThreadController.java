package org.firstinspires.ftc.teamcode.systems.modules.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.systems.modules.Executable;
import org.firstinspires.ftc.teamcode.systems.modules.interfaces.ThreadedInterface;

public abstract class ThreadController extends Thread implements ThreadedInterface {

    protected final Executable<Boolean> opModeIsActive;

    protected final Executable<Boolean> isStopRequested;

    protected volatile boolean isRunning = true;

    protected long sleepTime = 50;


    public ThreadController(Executable<Boolean> opModeIsActive, Executable<Boolean> isStopRequested) {
        this.isStopRequested = isStopRequested;
        this.opModeIsActive = opModeIsActive;
    }


    public final void end() {
        onEnd();
        this.isRunning = false;
        this.interrupt();
    }


    protected abstract void onEnd();

    final public void run() {
        try {
            while (isRunning && !isStopRequested.call()) {
                this.threadMain();
                Thread.sleep(sleepTime);
            }
        } catch (InterruptedException ignored) {
            RobotLog.d("Thread: " + Thread.currentThread().getName() + " has been interrupted");
        } catch (Throwable t) {
            RobotLog.addGlobalWarningMessage("Side Ran Thread Crashed because thrown Throwable: " + t.getClass());
            RobotLog.dd("Failure", t, "Fix Me");
        }

    }

    public final boolean isRunning() {
        return this.isRunning;
    }




}
