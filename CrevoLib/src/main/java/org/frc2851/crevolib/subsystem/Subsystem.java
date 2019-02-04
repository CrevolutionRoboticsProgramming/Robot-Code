package org.frc2851.crevolib.subsystem;

import org.frc2851.crevolib.Logger;

import java.util.Vector;

/**
 * Abstract class that defines subsystem behavior. An example of the subsystem may be a drivetrain or a shooter.
 * A subsystem subclass should contain all code that manipulates physical subsystems.
 */
public abstract class Subsystem
{
    private String _name;
    private Command _command,
            _defaultCommand = getDefaultCommand();
    private CommandGroup mAuxilaryCommandGroup = new CommandGroup();
    private boolean _isCommandInit, _isDefaultCommandInit;
    private CommandState mDefaultState = new CommandState(),
            mAuxilaryState = new CommandState();

    private Vector<Command> commandQueue = new Vector<>();

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
        _name = name;
    }

    public void setCommmandGroup(Command... commands) {
        mAuxilaryCommandGroup = new CommandGroup(commands);
        if (mAuxilaryCommandGroup.getSize() > 0) mAuxilaryState.isNull = false;
        mAuxilaryState.isInit = false;
        mAuxilaryState.isFinished = false;
    }

    synchronized void runCommand()
    {
        if (_defaultCommand != null && initCommand(_defaultCommand, mDefaultState.isInit))
                _defaultCommand.update(); // Default command does not stop!!!

        Command auxCommand = (mAuxilaryCommandGroup == null || mAuxilaryCommandGroup.getSize() == 0) ? null : mAuxilaryCommandGroup.getCommand();
        if (auxCommand != null) {
            initCommand(mAuxilaryCommandGroup.getCommand(), mAuxilaryState.isInit);

            if (!auxCommand.isFinished()) {
                auxCommand.update();
            } else {
                auxCommand.stop();
                if (!mAuxilaryCommandGroup.nextCommand()) {
                    log(mAuxilaryCommandGroup.toString() + " completed, is now empty", Logger.LogLevel.DEBUG);
                    mAuxilaryCommandGroup = null;
                    mAuxilaryState.isNull = true;
                } else {
                    mAuxilaryState.isInit = false;
                    mAuxilaryState.isFinished = false;
                }
            }
        }
    }

    private boolean initCommand(Command command, boolean isInit)
    {
        boolean isDefault = command == _defaultCommand;
        if (!isInit) {
            if (!command.init()) {
                if (isDefault) {
                    Logger.println("Could not initialize default command [" + command.getName() + "], setting default command to null", Logger.LogLevel.ERROR);
                    _defaultCommand = null;
                    return false;
                } else {
                    Logger.println("Could not initialize command: " + _command.getName(), Logger.LogLevel.ERROR);
                    _command = null;
                    return false;
                }
            } else {
                if (isDefault) mDefaultState.isInit = true;
                else mAuxilaryState.isInit = true;
            }

            if (isDefault) mDefaultState.isInit = true;
            else mAuxilaryState.isInit = true;
        }
        return true;
    }

    /**
     * Returns true if the subsystem has a command currently running.
     * @return {@code true} when the subsystem is active
     */
    public boolean isSubsystemActive() { return _command != null; }

    @Override
    public String toString() { return _name; }

    protected void log(String message, Logger.LogLevel level) {
        Logger.println("[" + _name + "] " + message, level);
    }

    public CommandState getDefaultCommandState() { return mDefaultState; }
    public CommandState getAuxilaryCommandState() { return mAuxilaryState; }
}
