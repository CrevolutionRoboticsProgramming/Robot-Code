package org.frc2851.crevolib.utilities;

import edu.wpi.first.wpilibj.DriverStation;

import java.io.File;

/**
 * Logs events
 */
public class Logger
{
    private static LogLevel logLevel = LogLevel.DEBUG;
    private static final String LOG_PATH = "/home/lvuser/logs/";

    /**
     * Stores enumerations representing three levels of logging. Each level has its own integer and prefix.
     */
    public enum LogLevel
    {
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

    private Logger()
    {
    }

    /**
     * Sets the level to log at
     *
     * @param level Level to log at
     */
    public static void setLogLevel(LogLevel level)
    {
        logLevel = level;
    }

    /**
     * Starts the Logger class
     */
    // TODO: Autogenerate unique log file name (timestamp-day.csv)
    public static void start()
    {
//        try {
//            _logFile = new File(LOG_PATH + "log.csv");
//            new FileWriter(_logFile).close();
//        } catch (IOException e) {
//            System.out.println("Unable to create log file");
//        }
    }

    /**
     * Prints the message at the specified level
     *
     * @param message Message to print
     * @param level   Level to print at
     */
    public static void println(String message, LogLevel level)
    {
        if (level.LEVEL < Logger.logLevel.LEVEL)
        {
            return;
        }

        message = level.MESSAGE_PREFIX + message;
//        FileWriter fw;
//        try
//        {
//            fw = new FileWriter(_logFile, true);
//            fw.append("\n" + message + ",");
//            fw.close();
//        } catch (IOException e) {
//            DriverStation.reportWarning("Logger Encountered an IOException", false);
//        }

        if (level == LogLevel.ERROR) DriverStation.reportError(message, false);
        if (level == LogLevel.WARNING) DriverStation.reportWarning(message, false);
        else System.out.println(message);
    }
}
