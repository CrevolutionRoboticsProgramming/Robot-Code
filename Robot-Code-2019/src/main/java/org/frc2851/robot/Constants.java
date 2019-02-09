package org.frc2851.robot;

import org.frc2851.robot.subsystems.DriveTrain;

/**
 * creates a class to Store and use Constants
 */
public class Constants
{
    private static Constants mInstance;

    /**
     * Gets the Instance state of the Constance class
     * @return The instance.
     */
    public static Constants getInstance() {
        if (mInstance == null) mInstance = new Constants();
        return mInstance;
    }

    /**
     * Runs the Constants class.
     */
    private Constants() { }

    public boolean singleControllerMode = false;

    // Drivetrain
    public final int leftDriveMaster = 1;
    public final int leftDriveSlaveA = 2;
    public final int leftDriveSlaveB = 3;
    public final int rightDriveMaster = 4;
    public final int rightDriveSlaveA = 5;
    public final int rightDriveSlaveB = 6;

    public final int pigeonID = 42;

    public final DriveTrain.DriveGear defaultDriveGear = DriveTrain.DriveGear.HIGH;

    // Elevator
    public final int elevatorMaster = 7;
    public final int elevatorSlave = 8;

    // Climber
    public final int gorillaMaster = 9;
    public final int gorillaSlave = 10;
    public final int screwMaster = 11;

    // Intake
    public final int intakeMaster = 0;

    public final int rollerClawTalon=0;

    public final int talonTimeout = 20;
}
