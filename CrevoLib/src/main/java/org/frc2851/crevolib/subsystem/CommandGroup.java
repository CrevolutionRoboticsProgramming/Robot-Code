package org.frc2851.crevolib.subsystem;

import java.util.Vector;

public class CommandGroup
{
    private Vector<Command> mCommands = new Vector<>();
    private final int kSize;
    private int index = 0;
    private boolean isEnabled = true;

    public CommandGroup(Command... commands) {
        if (commands != null) {
            for (Command c : commands) {
                if (c == null) continue;
                mCommands.add(c);
            }
            kSize = mCommands.size();
        } else {
            kSize = 0;
        }
    }

    Command getCommand() {
        return (isEnabled && kSize != 0) ? mCommands.get(index) : null;
    }

    public boolean nextCommand() {
        if (index + 1 < kSize) {
            index++;
            return true;
        }
        return false;
    }

    public void stop() {
        isEnabled = false;
    }

    public void resume() {
        isEnabled = true;
    }

    public void reset() {
        index = 0;
        isEnabled = true;
    }

    public int getSize() { return kSize; }

    @Override
    public String toString()
    {
        StringBuilder sb = new StringBuilder();
        sb.append("CommandGroup[");
        if (index == 0) {
            sb.append("null");
        } else {
            for (Command c : mCommands) {
                sb.append(c.getName());
                sb.append(", ");
            }
        }
        sb.append("]");
        return sb.toString();
    }
}
