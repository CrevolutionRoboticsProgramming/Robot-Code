package org.frc2851.crevolib.motion;

import org.frc2851.crevolib.Logger;

import java.io.*;
import java.util.ArrayList;

/**
 * Motion profile data structure
 */
public class MotionProfile
{
    private ArrayList<MotionProfilePoint> mPoints = new ArrayList<>();
    private String mName;

    /**
     * Creates a motion profile from a given file. That file has the count per feet as its file extension.
     *
     * @param file The file containing the motion profile data
     * @throws InvalidMotionProfileException Thrown when given an incorrectly formatted csv, when the csv is not found,
     *                                   and when the file can't be read (check file permissions)
     */
    public MotionProfile(File file) throws InvalidMotionProfileException
    {
        // Divides the mName of the file by periods and reads in the last extension as its cpf
        String[] strings = file.getName().split("\\.");
        int cpf = Integer.parseInt(strings[strings.length - 1]);
        mName = strings[0];
        Logger.println("Reading Motion Profile: " + mName + "(" + cpf + ")", Logger.LogLevel.DEBUG);
        try
        {
            BufferedReader csvReader = new BufferedReader(new FileReader(file));
            boolean isHeader = true;
            for (String str = csvReader.readLine(); str != null; str = csvReader.readLine())
            {
                if (isHeader)
                {
                    isHeader = false;
                    continue;
                }

                String[] vals = str.split(",");
                if (vals.length != 8)
                {
                    Logger.println("CSV[" + file.getName() + "]: Improper element length", Logger.LogLevel.ERROR);
                    throw new InvalidMotionProfileException();
                }

                double pos = Double.parseDouble(vals[3]);
                double vel = Double.parseDouble(vals[4]);
                int dt = Integer.parseInt(vals[0]);
                double heading = Double.parseDouble(vals[7]);
                mPoints.add(new MotionProfilePoint(pos, vel, dt, heading, cpf));
            }
        } catch (FileNotFoundException e)
        {
            Logger.println("Motion profile not found: " + file.getName(), Logger.LogLevel.ERROR);
            throw new InvalidMotionProfileException();
        } catch (IOException e)
        {
            Logger.println("Motion profile could not be read: " + file.getName(), Logger.LogLevel.ERROR);
            throw new InvalidMotionProfileException();
        }
    }

    /**
     * Returns a Vector of Motion Profile Points
     *
     * @return Vector of points
     */
    public ArrayList<MotionProfilePoint> getPoints()
    {
        return mPoints;
    }

    /**
     * Returns the number of points in the motion profile
     *
     * @return Number of points
     */
    public int getSize()
    {
        return mPoints.size();
    }

    /**
     * Returns the name of the motion profile
     *
     * @return Name
     */
    public String getName()
    {
        return mName;
    }
}
