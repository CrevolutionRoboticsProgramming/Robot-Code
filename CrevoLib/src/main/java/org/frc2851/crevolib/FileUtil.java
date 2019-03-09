package org.frc2851.crevolib;

import java.io.File;
import java.util.ArrayList;

/**
 * A class that contains a few helpful functions for working with files
 * @author Jason Deal
 */
public class FileUtil
{
    /**
     * A function that returns a list of all files in a directory. It does not return subdirectories.
     * @param path The absolute directory of the folder
     * @param recursive Search subdirectories
     * @return An array list containing all of the files
     */
    public static ArrayList<File> getFiles(String path, boolean recursive)
    {
        ArrayList<File> out = new ArrayList<>();
        if (recursive) {
            getFiles(new File(path), out);
        } else {
            File[] files = new File(path).listFiles();
            if (files != null)
                for (File f : files)
                    if (f.isFile())
                        out.add(f);
        }
        return out;
    }

    /**
     * Writes a list of Files to the provided ArrayList from the given directory and all subdirectories
     * @param dir The directory to search in
     * @param out The ArrayList to write to
     */
    private static void getFiles(File dir, ArrayList<File> out) {
        File[] files = dir.listFiles();
        if (files != null) {
            for (File f : files) {
                if (f.isDirectory())
                    getFiles(f, out);
                else if (f.isFile())
                    out.add(f);
            }
        }
    }
}
