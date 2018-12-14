package org.frc2851.crevolib;

import edu.wpi.first.wpilibj.DriverStation;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class Logger
{
    private static final LogLevel LEVEL = LogLevel.DEBUG;
    private static final String LOG_PATH = "/home/lvuser/logs/";

    public enum LogLevel {
        DEBUG(0, "D: "), WARNING(1, "W: "), ERROR(2, "E: ");

        private final int LEVEL;
        private final String MESSAGE_PREFIX;
        LogLevel(int level, String messagePrefix)
        {
            LEVEL = level;
            MESSAGE_PREFIX = messagePrefix;
        }
    }

    private static File _logFile;

    private Logger() { }

    // TODO: Autogenerate unique log file name (timestamp-day.csv)
    static void start() {
        try {
            _logFile = new File(LOG_PATH + "log.csv");
            new FileWriter(_logFile).close();
        } catch (IOException e) {
            System.out.println("Unable to create log file");
        }
    }

    public static void println(String message, LogLevel level)
    {
        if (level.LEVEL < LEVEL.LEVEL) { return; }

        FileWriter fw;
        try
        {
            fw = new FileWriter(_logFile, true);
            fw.append("\n" + level.MESSAGE_PREFIX + message + ",");
            fw.close();
        } catch (IOException e) {
            DriverStation.reportWarning("Logger Encountered an IOException", false);
        }

        if (level == LogLevel.ERROR) DriverStation.reportError(message, false);
        if (level == LogLevel.WARNING) DriverStation.reportWarning(message, false);
        else System.out.println(message);
    }
}
