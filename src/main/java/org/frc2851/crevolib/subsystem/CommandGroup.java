package org.frc2851.crevolib.subsystem;

import java.util.Vector;

public class CommandGroup
{
    private Vector<Command> mCommands = new Vector<>();
    private final int kSize;
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
            kSize = mCommands.size();
        } else
        {
            kSize = 0;
        }
    }

    Command getCommand()
    {
        return (mEnabled && kSize != 0) ? mCommands.get(mIndex) : null;
    }

    public boolean nextCommand()
    {
        if (mIndex + 1 < kSize)
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
        return kSize;
    }

    @Override
    public String toString()
    {
        StringBuilder sb = new StringBuilder();
        sb.append("CommandGroup[");
        if (kSize == 0)
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
