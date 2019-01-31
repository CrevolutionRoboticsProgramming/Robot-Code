package org.frc2851.robot;

import org.frc2851.robot.subsystems.DriveTrain;

public class Constants
{
    private static Constants mInstance;
    public static Constants getInstance() {
        if (mInstance == null) mInstance = new Constants();
        return mInstance;
    }

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
    public final int elevatorCanifier = 11;

    public final double elevatorRawMultiplier = 0.8;

    public final int elevatorMaximumVelocity = 0;
    public final int elevatorMaximumAcceleration = 0;
    public final int elevatorAllowedClosedLoopError = 128;

    // Climber
    public final int gorillaMaster = 9;
    public final int gorillaSlave = 10;

    public final int talonTimeout = 20;
}
