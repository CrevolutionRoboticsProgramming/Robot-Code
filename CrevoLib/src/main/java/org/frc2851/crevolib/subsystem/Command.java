<<<<<<< HEAD
package org.frc2851.crevolib.subsystem;

/**
 * An iterative set of instructions for a Subsystem to follow
 */
public interface Command
{
    /**
     * Returns the name of the command
     * @return Name of the command
     */
    String getName();

    /**
     * Returns true when the command is complete. Once this is true, the Subsystem will stop the command.
     * @return {@code true} if the command is complete
     */
    boolean isFinished();

    /**
     * Runs once when the command first starts
     */
    boolean init();

    /**
     * Runs iteratively at 200hz
     */
    void update();

    /**
     * Called either when the command completes or it is stopped. It will be stopped if another command is set before
     * the current command finishes.
     */
    void stop();
=======
package org.frc2851.crevolib.subsystem;

/**
 * An iterative set of instructions for a Subsystem to follow
 */
public interface Command
{
    /**
     * Returns the name of the command
     * @return Name of the command
     */
    String getName();

    /**
     * Returns true when the command is complete. Once this is true, the Subsystem will stop the command.
     * @return {@code true} if the command is complete
     */
    boolean isFinished();

    /**
     * Runs once when the command first starts
     */
    boolean init();

    /**
     * Runs iteratively at 200hz
     */
    void update();

    /**
     * Called either when the command completes or it is stopped. It will be stopped if another command is set before
     * the current command finishes.
     */
    void stop();
>>>>>>> 291f823785993974c90dbca1d834dba43d07ee46
}