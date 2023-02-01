package org.frc2851.crevolib.auton;

/*
 *  Taken from team 251's 2017 robot
 */

public interface Action {
    boolean init(); // Initialization method
    void update(); // Runs iteratively until action is complete
    boolean isFinished(); // Returns true if the action has completed
    void stop(); // Runs when the action is complete

}
