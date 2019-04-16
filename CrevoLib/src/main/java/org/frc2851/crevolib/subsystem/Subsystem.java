package org.frc2851.crevolib.subsystem;

import org.frc2851.crevolib.utilities.Logger;

/**
 * Abstract class that defines subsystem behavior. An example of the subsystem may be a drivetrain or a shooter.
 * A subsystem subclass should contain all code that manipulates physical subsystems.
 */
public abstract class Subsystem
{
    private String mName;
    private Command mDefaultCommand = getDefaultCommand();
    private CommandGroup mAuxiliaryCommandGroup = null;
    private CommandState mDefaultState = new CommandState("Default Command"),
            mAuxiliaryState = new CommandState("Auxilary Command");

    boolean disableSubsystem = false;

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

    public synchronized void restartDefaultCommand()
    {
        if (mDefaultState.isNull)
        {
            mDefaultCommand = getDefaultCommand();
            mDefaultState.isNull = false;
            mDefaultState.isInit = false;
            mDefaultState.isFinished = false;
        }
    }

    /**
     * Sets the group the Command is in
     * @param commands
     */
    public synchronized void setCommmandGroup(Command... commands) {
        mAuxiliaryCommandGroup = new CommandGroup(commands);
        if (mAuxiliaryCommandGroup.getSize() > 0) mAuxiliaryState.isNull = false;
        mAuxiliaryState.isInit = false;
        mAuxiliaryState.isFinished = false;

        log("Set aux command group: " + mAuxiliaryCommandGroup.toString(), Logger.LogLevel.DEBUG);
    }

    /**
     * Runs a Command, and logs if that command was successful.
     */
    synchronized void runCommand()
    {
        if (mDefaultCommand != null && initCommand(mDefaultCommand, mDefaultState) && !disableSubsystem) mDefaultCommand.update();

        if (mAuxiliaryCommandGroup != null)
        {
            Command auxCommand = mAuxiliaryCommandGroup.getCommand();
            if (auxCommand != null && (initCommand(auxCommand, mAuxiliaryState)))
            {
                if (!auxCommand.isFinished())
                {
                    auxCommand.update();
                } else {
                    auxCommand.stop();
                    if (mAuxiliaryCommandGroup.nextCommand())
                    {
                        mAuxiliaryState.isInit = false;
                    } else {
                        log(mAuxiliaryCommandGroup.toString() + " completed", Logger.LogLevel.DEBUG);
                        mAuxiliaryCommandGroup = null;
                        mAuxiliaryState.isNull = true;
                    }
                }
            } else {
                log(mAuxiliaryCommandGroup.toString() + "was unsuccessful", Logger.LogLevel.ERROR);
                mAuxiliaryCommandGroup = null;
                mAuxiliaryState.isNull = true;
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

        log("Started command: " + command.getName(), Logger.LogLevel.DEBUG);
        state.isInit = true;
        return true;
    }

    public synchronized void stopSubsystem()
    {
        log("Disabling subsystem", Logger.LogLevel.DEBUG);
        stopAuxiliaryCommand();
        if (mDefaultCommand == null) return;
        mDefaultCommand.stop();
        mDefaultState = null;
        mDefaultState.isNull = true;
    }

    /**
     * Stops the current Auxilary Command
     */
    public synchronized void stopAuxiliaryCommand()
    {
        if (mAuxiliaryCommandGroup == null || mAuxiliaryCommandGroup.getSize() == 0) return;
        Command c = mAuxiliaryCommandGroup.getCommand();
        log("Stopping " + mAuxiliaryCommandGroup.toString() + " on command " + c.getName(), Logger.LogLevel.DEBUG);
        c.stop();
        mAuxiliaryCommandGroup = null;
        mAuxiliaryState.isNull = true;
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
    public boolean getAuxiliaryCommandActivity()
    {
        return !mAuxiliaryState.isNull;
    }

    public String getName()
    {
        return mName;
    }
}
