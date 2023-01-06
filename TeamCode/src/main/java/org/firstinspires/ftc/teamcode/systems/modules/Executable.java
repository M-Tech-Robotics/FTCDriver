package org.firstinspires.ftc.teamcode.systems.modules;

/**
 * This interface provides a method of passing lambda objects
 */
public interface Executable<ReturnType> {

    /**
     * This calls the internal function
     *
     * @return The type of object specified by ReturnType
     */
    ReturnType call();

}
