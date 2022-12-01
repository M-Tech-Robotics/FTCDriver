package org.firstinspires.ftc.teamcode.systems.Utils.Objects;


/**
 * A class for controlling toggleable button states
 */
public class ToggleButton {
    /** VARIABLES **/
    public boolean state = false;
    public boolean lastButton = false;

    /** MAIN **/
    public ToggleButton(){}


    public void update(boolean button) {
        if(button && !lastButton){
            state = !state;
        }
        
        lastButton = button;
    }

    public boolean getState() {
        return state;
    }
}