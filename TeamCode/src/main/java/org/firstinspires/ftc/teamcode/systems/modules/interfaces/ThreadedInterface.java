package org.firstinspires.ftc.teamcode.systems.modules.interfaces;

public interface ThreadedInterface extends Runnable {
    /**
     * It is the main function of the thread
     *
     * @throws InterruptedException Throws if the thread is interrupted
     */
    void threadMain() throws InterruptedException;

    /**
     * Ends the life of this thread
     */
    void end();

    /**
     * Returns the running state of the thread
     *
     * @return true if running, false otherwise
     */
    boolean isRunning();
}
