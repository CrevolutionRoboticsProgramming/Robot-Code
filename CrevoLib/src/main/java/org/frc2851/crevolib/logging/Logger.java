package org.frc2851.crevolib.logging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Vector;

public class Logger extends Thread
{
    private final String LOG_DIR;

    private Vector<CSVFile> mFiles = new Vector<>();
    private Thread mThread = null;
    private boolean isEnabled = false;

    private static Logger mInstance = new Logger();

    private class CSVFile
    {
        private CSVWritable writableObj;
        private BufferedWriter writer;

        CSVFile(File file, CSVWritable writable) throws IOException
        {
            this.writableObj = writable;
            writer = new BufferedWriter(new FileWriter(file));
            writer.write(writable.getHeader());
        }

        void update() throws IOException
        {
            writer.write(writableObj.getCSV());
        }

        @Override
        public String toString() { return writableObj.getFileName(); }
    }

    private Logger()
    {
        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("HH-mm-ss");
        LOG_DIR = "/home/lvuser/logs/" + formatter.format(LocalDateTime.now());
    }

    public static Logger getInstance() { return mInstance; }

    public void addWritable(CSVWritable writable)
    {
        try {
            mFiles.add(new CSVFile(new File(LOG_DIR + writable.getFileName()), writable));
        } catch (IOException e) {
            DriverStation.reportError("Could not create writable[" + writable.getFileName() + "]", false);
        }
    }

    @Override
    public final void run()
    {
        while (isEnabled)
        {
            for (CSVFile file : mFiles) {
                try {
                    file.update();
                } catch (IOException e) {
                    DriverStation.reportError("Could not write to log[" + file.toString() + "]", false);
                }
            }
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public final void start()
    {
        if (mThread == null)
        {
            isEnabled = true;
            mThread = new Thread(this);
            mThread.start();
        }
    }
}
