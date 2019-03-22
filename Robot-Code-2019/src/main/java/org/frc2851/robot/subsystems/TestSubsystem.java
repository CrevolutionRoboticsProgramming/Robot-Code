package org.frc2851.robot.subsystems;

import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.crevolib.utilities.Logger;
import org.frc2851.robot.Robot;

public class TestSubsystem extends Subsystem
{
    public TestSubsystem() { super("Test Subsystem"); }

    @Override
    protected boolean init()
    {
        Controller c = new Controller(0);
        return true;
    }

    @Override
    public Command getDefaultCommand()
    {
        return new Command()
        {
            Controller c = new Controller(0);
            @Override
            public String getName()
            {
                return "Default Command";
            }

            @Override
            public boolean isFinished()
            {
                return false;
            }

            @Override
            public boolean init()
            {
                return false;
            }

            @Override
            public void update()
            {
                if (Robot.isRunning())
                {
                    if (!c.isControllerConnected())
                    {
                        log("Controller not connected...", Logger.LogLevel.ERROR);
                        stopSubsystem();
                        return;
                    }
                }
            }

            @Override
            public void stop()
            {

            }
        };
    }
}
