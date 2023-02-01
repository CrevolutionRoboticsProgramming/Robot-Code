package org.frc2851.crevolib.subsystem;

public class CommandState
{
    public boolean isFinished = false;
    public boolean isInit = false;
    public boolean isNull = false;

    private final String NAME;

    public CommandState(String name)
    {
        this.NAME = name;
    }


    @Override
    public String toString()
    {
        return "CommandState[" + NAME + "][Finished: " + isFinished + ", Initialized: " + isInit + ", Null: " + isNull + "]";
    }
}
