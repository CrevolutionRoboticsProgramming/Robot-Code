package org.frc2851.crevolib.motion;

import edu.wpi.first.wpilibj.DriverStation;

import java.io.*;
import java.util.Vector;

// Note: Non Jaci CSV file name format: name.csv.cpf (note: cpf is a number)

/**
 * Motion profile data structure
 */
public class MotionProfile
{
    private Vector<MotionProfilePoint> _points = new Vector<>();
    private String name = "unknown";

    /**
     * Creates a motion profile from a given file. That file has the count per feet as its file extension.
     * @param file The file containing the motion profile data
     * @throws BadMotionProfileException Thrown when given an incorrectly formatted csv, when the csv is not found,
     *                                   and when the file can't be read (check file permissions)
     */
    public MotionProfile(File file) throws BadMotionProfileException
    {
        // Divides the name of the file by periods and reads in the last extension as its cpf
        String[] strings = file.getName().split("\\.");
//        int cpf = Integer.parseInt(strings[strings.length - 1]);
        int cpf = 325;
        name = strings[0];
        try
        {
            BufferedReader csvReader = new BufferedReader(new FileReader(file));
            boolean isHeader = true;
            for (String str = csvReader.readLine(); str != null; str = csvReader.readLine())
            {
                if (isHeader) {
                    isHeader = false;
                    continue;
                }

                String[] vals = str.split(",");
                if (vals.length != 8)
                {
                    System.err.println("CSV[" + file.getName() + "]: Improper element length");
                    throw new BadMotionProfileException();
                }

                double pos = Double.parseDouble(vals[3]);
                double vel = Double.parseDouble(vals[4]);
                double dt = Double.parseDouble(vals[0]);
                double heading = Double.parseDouble(vals[7]);
                System.out.println("MP[" + pos + ", " + vel + ", " + heading + ", " + dt + "]");
                _points.add(new MotionProfilePoint(pos, vel, dt, heading, cpf));
            }
        } catch (FileNotFoundException e) {
            DriverStation.reportError("CSV[" + file.getName() + "]: File not found", false);
            throw new BadMotionProfileException();
        } catch (IOException e) {
            DriverStation.reportError("CSV[" + file.getName() + "]: Could not read file. Check file permissions.", false);
            throw new BadMotionProfileException();
        }
    }

    /**
     * Returns a Vector of Motion Profile Points
     * @return Vector of points
     */
    public Vector<MotionProfilePoint> getPoints() { return _points; }

    /**
     * Returns the number of points in the motion profile
     * @return Number of points
     */
    public int getSize() { return _points.size(); }

    /**
     * Returns the name of the motion profile
     * @return Name
     */
    public String getName() { return name; }
}
