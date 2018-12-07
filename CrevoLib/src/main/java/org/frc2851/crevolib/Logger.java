package org.frc2851.crevolib;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class Logger
{
    private static File file = new File("/home/lvuser/log");
    private Logger() { }

    public static void start() {
        try {
            new FileWriter(file).close();
        } catch (IOException e) {
            System.out.println("Unable to create log file");
        }
    }

    public static void println(String message)
    {
        FileWriter fw;
        try
        {
            fw = new FileWriter(file, true);
            fw.append("\n" + message);
            fw.close();
        } catch (IOException e) {
            System.err.println("Unable to Print Message");
        }

        System.out.println(message);
    }

    public static void printerr(String message)
    {
        FileWriter fw;
        try
        {
            fw = new FileWriter(file, true);
            fw.append("\n[ERROR!]: " + message);
            fw.close();
        } catch (IOException e) {
            System.err.println("Unable to Print Message");
        }

        System.out.println(message);
    }
}
