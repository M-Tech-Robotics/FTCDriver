package org.firstinspires.ftc.teamcode.systems.modules.Actions;

public interface ActionInterface extends Runnable {
    void mainAction() throws InterruptedException;


    void end();


    boolean isRunning();
}
