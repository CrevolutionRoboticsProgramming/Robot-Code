package org.frc2851.crevolib.autonomous;

/**
 * A framework for an auton action.
 */
public interface Action
{
    /**
     * Called once when the auton first starts.
     */
    default void init() {}

    /**
     * Runs iteratively at 200hz.
     */
    void update();

    /**
     * Returns true when the auton completes
     * @return {@code true} when auton completes
     */
    boolean isFinished();

    /**
     * Returns the name of the auton
     * @return name
     */
    String getName();
}
