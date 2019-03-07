package org.frc2851.crevolib.subsystem;

import java.util.Vector;

public class CommandGroup
{
    private Vector<Command> mCommands = new Vector<>();
    private final int SIZE;
    private int mIndex = 0;
    private boolean mEnabled = true;

    public CommandGroup(Command... commands)
    {
        if (commands != null)
        {
            for (Command c : commands)
            {
                if (c == null) continue;
                mCommands.add(c);
            }
            SIZE = mCommands.size();
        } else
        {
            SIZE = 0;
        }
    }

    Command getCommand()
    {
        return (mEnabled && SIZE != 0) ? mCommands.get(mIndex) : null;
    }

    public boolean nextCommand()
    {
        if (mIndex + 1 < SIZE)
        {
            mIndex++;
            return true;
        }
        return false;
    }

    public void stop()
    {
        mEnabled = false;
    }

    public void resume()
    {
        mEnabled = true;
    }

    public void reset()
    {
        mIndex = 0;
        mEnabled = true;
    }

    public int getSize()
    {
        return SIZE;
    }

    @Override
    public String toString()
    {
        StringBuilder sb = new StringBuilder();
        sb.append("CommandGroup[");
        if (SIZE == 0)
        {
            sb.append("null");
        } else
        {
            for (int i = 0; i < mCommands.size(); i++)
            {
                sb.append(mCommands.elementAt(i).getName());
                if (i != mCommands.size() - 1) sb.append(", ");
            }
        }
        sb.append("]");
        return sb.toString();
    }
}
