package org.frc2851.crevolib.subsystem;

import org.frc2851.crevolib.Logger;

/**
 * Abstract class that defines subsystem behavior. An example of the subsystem may be a drivetrain or a shooter.
 * A subsystem subclass should contain all code that manipulates physical subsystems.
 */
public abstract class Subsystem
{
    private String mName;
    private Command mDefaultCommand = getDefaultCommand();
    private CommandGroup mAuxilaryCommandGroup = null;
    private CommandState mDefaultState = new CommandState(), mAuxilaryState = new CommandState();

    /**
     * Runs once when the subsystem first starts
     */
    protected abstract boolean init();

    /**
     * Returns the default teleop command for the subsystem
     * @return The teleop command
     */
    public abstract Command getDefaultCommand();

    /**
     * Creates the subsystem and sets the name of the subsystem (this is used in logging). Note that this is an
     * abstract class and this constructor should only be used as super in a subclass.
     * @param name The name of the subsystem
     */
    protected Subsystem(String name)
    {
        mName = name;
    }

    /**
     * Sets the group the Command is in
     * @param commands
     */
    public void setCommmandGroup(Command... commands) {
        mAuxilaryCommandGroup = new CommandGroup(commands);
        if (mAuxilaryCommandGroup.getSize() > 0) mAuxilaryState.isNull = false;
        mAuxilaryState.isInit = false;
        mAuxilaryState.isFinished = false;
    }

    /**
     * Runs a Command, and logs if that command was successful.
     */
    synchronized void runCommand()
    {
        if (mDefaultCommand != null && initCommand(mDefaultCommand, mDefaultState)) mDefaultCommand.update();

        if (mAuxilaryCommandGroup != null)
        {
            Command auxCommand = mAuxilaryCommandGroup.getCommand();
            if (auxCommand != null && initCommand(auxCommand, mAuxilaryState))
            {
                if (auxCommand.isFinished())
                {
                    auxCommand.update();
                } else {
                    auxCommand.stop();
                    if (!mAuxilaryCommandGroup.nextCommand())
                    {
                        log(mAuxilaryCommandGroup.toString() + " completed", Logger.LogLevel.DEBUG);
                        mAuxilaryCommandGroup = null;
                        mAuxilaryState.isNull = true;
                    }
                }
            } else {
                log(mAuxilaryCommandGroup.toString() + "was unsuccessful", Logger.LogLevel.ERROR);
                mAuxilaryCommandGroup = null;
                mAuxilaryState.isNull = true;
            }
        }
    }

    /**
     *Initilizes a new command, along with a state.
     * @param command Uses a Command
     * @param state  Uses a State
     * @return true if the command was successfully initialized else returns false
     */
    private boolean initCommand(Command command, CommandState state)
    {
        if (state.isInit) return true;

        if (!command.init())
        {
            log("Could not initialize command: " + command.getName(), Logger.LogLevel.ERROR);
            command = null;
            state.isNull = true;
            return false;
        }

        state.isInit = true;
        return true;
    }

    /**
     * Stops the current Auxilary Command
     */
    public synchronized void stopAuxilaryCommand()
    {
        if (mAuxilaryCommandGroup == null) return;
        Command c = mAuxilaryCommandGroup.getCommand();
        c.stop();
        mAuxilaryCommandGroup = null;
        mAuxilaryState.isNull = true;
    }

    /**
     * converts a string to a mName for use
     * @return The mName
     */
    @Override
    public String toString()
    {
        return mName;
    }

    /**
     * Logs a message prepended with the subsystem name
     * @param message Message of String
     * @param level Log level
     */
    protected void log(String message, Logger.LogLevel level)
    {
        Logger.println("[" + mName + "] " + message, level);
    }

    /**
     * Returns true if the default command is active
     * @return Is command active
     */
    public boolean getDefaultCommandActivity()
    {
        return !mDefaultState.isNull;
    }

    /**
     * Returns true if the auxilary command is active
     * @return Is command active
     */
    public boolean getAuxilaryCommandActivity()
    {
        return !mAuxilaryState.isNull;
    }

    public String getName()
    {
        return mName;
    }
}
