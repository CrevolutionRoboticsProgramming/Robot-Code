package org.frc2851.robot.subsystems;

import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Robot;

public class ExampleSubsystem extends Subsystem
{
    private static ExampleSubsystem instance = new ExampleSubsystem();

    private ExampleSubsystem()
    {
        super("Example Subsystem");
    }

    public static ExampleSubsystem getInstance()
    {
        return instance;
    }

    @Override
    protected boolean init()
    {
        return true;
    }

    @Override
    public Command getDefaultCommand()
    {
        return new Command()
        {
            @Override
            public String getName()
            {
                return "Teleop";
            }

            @Override
            public boolean isFinished()
            {
                return false;
            }

            @Override
            public boolean init()
            {
                return true;
            }

            @Override
            public void update()
            {
                if (Robot.isRunning())
                {
                }
            }

            @Override
            public void stop()
            {

            }
        };
    }
}
