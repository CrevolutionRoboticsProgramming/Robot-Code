package org.frc2851.crevolib.logging;

/**
 * Interface for a CSVWritable object
 */
public interface CSVWritable
{
    /**
     * Returns the header, which is a list of variables that are being logged. Drivetrain in 2019-Vulcan has a good example
     * @return The header
     */
    String getHeader();

    /**
     * Returns one line of the csv. Again view drivetrain for an example.
     * @return The csv
     */
    String getCSV();

    /**
     * Returns the name of the CSV file. This will be the base of the name for the log file.
     * @return The name
     */
    String getFileName();
}
