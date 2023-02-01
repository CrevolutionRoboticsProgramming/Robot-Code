package org.frc2851.crevolib.logging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;

import java.io.*;
import java.util.Vector;
import java.util.concurrent.locks.ReentrantLock;

public class Logger
{
    private LogLevel mLogLevel = LogLevel.DEBUG;
    private final String DIR = "/home/lvuser/logs/events/";

    private Notifier mNotifier;
    private ReentrantLock mMutex = new ReentrantLock();

    private BufferedWriter mWriter = null;

    private Vector<LogMessage> mLogBuffer = new Vector<>();

    public Logger(String name)
    {
        try
        {
            File f = new File(DIR + name + "_" + getIndex(new File(DIR + "log.idx")) + ".log");
            if (f.exists()) f.delete();
            mWriter = new BufferedWriter(new FileWriter(f));
            mNotifier = new Notifier(this::writeLog);
            mNotifier.startPeriodic(0.05);
        } catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    public void addMessage(LogMessage message)
    {
        while (mMutex.isLocked()) ;
        mLogBuffer.add(message);
    }

    private void writeLog()
    {
        mMutex.lock();
        StringBuilder builder = new StringBuilder();
        for (LogMessage message : mLogBuffer)
        {
            builder.append(message.toString() + "\n");
            printToDS(message.toString(), message.getLogLevel());
        }
        mLogBuffer.clear();
        mMutex.unlock();
        if (mWriter != null)
        {
            try
            {
                mWriter.append(builder.toString());
                mWriter.flush();
            } catch (IOException e)
            {
                e.printStackTrace();
            }
        }
    }

    private void printToDS(String message, LogLevel level)
    {
        if (level.ordinal() <= mLogLevel.ordinal())
        {
            if (level == LogLevel.ERROR) DriverStation.reportError(message, false);
            else if (level == LogLevel.WARNING) DriverStation.reportWarning(message, false);
            else System.out.println(message);
        }
    }

    // TODO: Cleanup, its ugly but it works
    private static int getIndex(File indexFile)
    {
        try
        {
            if (indexFile.createNewFile())
            {
                BufferedWriter writer = new BufferedWriter(new FileWriter(indexFile));
                writer.append("1");
                writer.flush();
                writer.close();
                return 0;
            }

            BufferedReader reader = new BufferedReader(new FileReader(indexFile));
            int index = Integer.parseInt(reader.readLine());
            reader.close();
            indexFile.delete();
            indexFile.createNewFile();
            BufferedWriter writer = new BufferedWriter(new FileWriter(indexFile));
            writer.write(Integer.toString(index + 1));
            writer.flush();
            writer.close();
            return index;
        } catch (IOException e)
        {
            e.printStackTrace();
        }
        return -1;
    }

    public static void main(String... args)
    {
        Logger l = new Logger("test");
        l.addMessage(new LogMessage("Test", "Hello", LogLevel.DEBUG));
    }
}