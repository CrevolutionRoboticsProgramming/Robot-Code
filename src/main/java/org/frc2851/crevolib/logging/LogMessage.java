package org.frc2851.crevolib.logging;

import edu.wpi.first.wpilibj.Timer;

import java.text.DecimalFormat;

public class LogMessage
{
    private final String tag;
    private final String timeStamp;
    private final String message;
    private final LogLevel level;

    public LogMessage(String tag, String message, LogLevel level)
    {
        DecimalFormat timeStampFormat = new DecimalFormat(".000");
        this.tag = tag;
        this.timeStamp = timeStampFormat.format(Timer.getFPGATimestamp());
        this.message = message;
        this.level = level;
    }

    public LogLevel getLogLevel()
    {
        return level;
    }

    @Override
    public String toString()
    {
        StringBuilder sb = new StringBuilder();
        sb.append(level.getTag());
        sb.append("[" + tag + " | " + timeStamp + "] ");
        sb.append(message);
        return sb.toString();
    }
}
