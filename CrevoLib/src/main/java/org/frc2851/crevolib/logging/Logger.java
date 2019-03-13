package org.frc2851.crevolib.logging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Vector;
import java.util.concurrent.locks.ReentrantLock;

public class Logger
{
    private Notifier mNotifier;
    private ReentrantLock mMutex = new ReentrantLock();

    private final BufferedWriter mWriter;

    private Vector<LogMessage> mLogBuffer = new Vector<>();

    public Logger(String path) throws IOException
    {
        File f = new File(path);
        if (f.exists()) f.delete();
        mWriter = new BufferedWriter(new FileWriter(f));
    }

    public void addMessage(LogMessage message)
    {
        while (mMutex.isLocked());
        mLogBuffer.add(message);
        mNotifier = new Notifier(this::writeLog);
        mNotifier.startPeriodic(0.05);
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
        try
        {
            mWriter.append(builder.toString());
        } catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    private void printToDS(String message, LogLevel level)
    {
        if (level == LogLevel.ERROR) DriverStation.reportError(message, false);
        else if (level == LogLevel.WARNING) DriverStation.reportWarning(message, false);
        else System.out.println(message);
    }
}
