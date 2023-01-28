package org.frc2851.crevolib.logging;

public enum LogLevel
{
    ERROR, WARNING, DEBUG;

    public String getTag()
    {
        return "<" + this.name() + ">";
    }
}
