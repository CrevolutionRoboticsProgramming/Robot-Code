package org.frc2851.crevolib.subsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.frc2851.crevolib.Logger;

/**
 * Abstract class that defines subsystem behavior. An example of the subsystem may be a drivetrain or a shooter.
 * A subsystem subclass should contain all code that manipulates physical subsystems.
 */
public abstract class Subsystem
{
    private String _name;
    private Command _command, _defaultCommand = null;
    private boolean _isCommandInit, _isDefaultCommandInit;
    private CommandState mPrimaryState = new CommandState(),
            mSecondaryState = new CommandState();

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
    protected Subsystem(String name) { _name = name; }

    /**
     * Sets the current command. If there is another command running, this will stop the current command.
     * @param command The command to be set
     */
    public synchronized void setCommand(Command command)
    {
//        if (command != null) Logger.println("[" + _name + "] SetCommand: " + _name + ", " + command.getName(), Logger.LogLevel.DEBUG);
//        else Logger.println("[" + _name + "] Command set to Idle", Logger.LogLevel.DEBUG);
//        if (_command != null) _command.stop();
//        _command = command;
        setCommand(command, _command);
        _isCommandInit = false;
    }

    public void setDefaultCommand(Command command)
    {
//        if (_defaultCommand != null) Logger.println("[" + _name + "] SetDefaultCommand: " + _name + ", " + command.getName(), Logger.LogLevel.DEBUG);
//        else Logger.println("[" + _name + "] Default Command set to Idle", Logger.LogLevel.DEBUG);
//        if (_defaultCommand != null) _command.stop();
        setCommand(command, _defaultCommand);
        _isDefaultCommandInit = false;
        mPrimaryState.isInit = false;
        mPrimaryState.isFinished = false;
    }

    private void setCommand(Command newCommand, Command oldCommand)
    {
        if (newCommand != null) Logger.println("[" + _name + "] SetCommand: " + _name + ", " + newCommand.getName(), Logger.LogLevel.DEBUG);
        else Logger.println("[" + _name + "] Command set to Idle", Logger.LogLevel.DEBUG);
        if (oldCommand != null) _command.stop();
        oldCommand = newCommand;
    }

    synchronized void runCommand()
    {
        if (_command != null)
        {
            initCommand(_command, _isCommandInit);

            if (!_command.isFinished()) {
                _command.update();
            } else {
                mSecondaryState.isFinished = true;
                _command.stop();
                _command = null;
            }
        } else {
            mSecondaryState.isFinished = true;
            mSecondaryState.isInit = true;
        }

        if (_defaultCommand != null)
        {
            initCommand(_defaultCommand, _isDefaultCommandInit);

            _defaultCommand.update(); // Default command does not stop!!!
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
                if (isDefault) mPrimaryState.isInit = true;
                else mSecondaryState.isInit = true;
            }

            if (isDefault) _isDefaultCommandInit = true;
            else _isCommandInit = true;
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

    public static void runCommandGroup(Subsystem subsystem, Command... command)
    {
        for (Command c : command)
        {
            subsystem.setCommand(c);
            while (subsystem.isSubsystemActive());
        }
    }

    public CommandState getDefaultCommandState() { return mPrimaryState; }
    public CommandState getSecondaryCommandState() { return mSecondaryState; }
}
