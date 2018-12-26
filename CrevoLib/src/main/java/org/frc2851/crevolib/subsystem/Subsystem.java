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
    private Command _command;
    private boolean _isCommandInit;

    /**
     * Runs once when the subsystem first starts
     */
    protected abstract boolean init();

    /**
     * Returns the default teleop command for the subsystem
     * @return The teleop command
     */
    public abstract Command getTeleopCommand();

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
        if (command != null) Logger.println("[" + _name + "] SetCommand: " + _name + ", " + command.getName(), Logger.LogLevel.DEBUG);
        else Logger.println("[" + _name + "] Command set to Idle", Logger.LogLevel.DEBUG);
        if (_command != null) _command.stop();
        _command = command;
        _isCommandInit = false;
    }

    synchronized void runCommand()
    {
        if (_command != null)
        {
            if (!_isCommandInit)
            {
                if (!_command.init())
                {
                    Logger.println("Could not initialize command: " + _command.getName(), Logger.LogLevel.ERROR);
                    _command = null;
                    return;
                }
                _isCommandInit = true;
            }

            if (!_command.isFinished()) {
                _command.update();
            } else {
                _command.stop();
                _command = null;
            }
        }
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
}
