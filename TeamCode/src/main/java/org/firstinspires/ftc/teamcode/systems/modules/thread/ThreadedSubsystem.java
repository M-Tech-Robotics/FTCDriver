package org.firstinspires.ftc.teamcode.systems.modules.thread;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.systems.modules.Executable;

/**
 * This is a template for all subsystems that need threading, provides some thread safety
 */
public abstract class ThreadedSubsystem extends Thread implements ThreadedInterface {

    /**
     * A Lambda object that allows this class to check that the OpMode is active
     */
    protected final Executable<Boolean> opModeIsActive;
    /**
     * A Lambda object that allows this class to check the stop requested condition of the OpMode
     */
    protected final Executable<Boolean> isStopRequested;
    /**
     * A boolean that controls if the thread is running
     */
    protected volatile boolean isRunning = true;
    /**
     * The time in mills that the thread sleeps for after every call of {@link #threadMain()}
     */
    protected long sleepTime = 50;

    /**
     * A constructor that instantiates the Executable Objects, they allow the thread to end with the OpMode
     *
     * @param isStopRequested A Executable object wrapped around {@link LinearOpMode#isStopRequested()}
     * @param opModeIsActive  A Executable object wrapped around {@link LinearOpMode#opModeIsActive()}
     */
    public ThreadedSubsystem(Executable<Boolean> opModeIsActive, Executable<Boolean> isStopRequested) {
        this.isStopRequested = isStopRequested;
        this.opModeIsActive = opModeIsActive;
    }

    /**
     * Ends the life of this thread
     */
    public final void end() {
        onEnd();
        this.isRunning = false;
        this.interrupt();
    }

    /**
     * Cleans up before the thread spools down
     */
    protected abstract void onEnd();

    /**
     * This is the method where the thread starts
     */
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

    /**
     * Returns the running state of the thread
     *
     * @return true if running, false otherwise
     */
    public final boolean isRunning() {
        return this.isRunning;
    }


}
