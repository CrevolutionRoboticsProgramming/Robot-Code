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

    public final int pcmID = 0;

    // Elevator
    public final int elevatorMaster = 7;
    public final int elevatorSlave = 8;

    // Climber
    public final int gorillaMaster = 9;
    public final int gorillaSlave = 10;

    // Intake
    public final int intakeMaster = 0;
    public final int intakeForward = 0;
    public final int intakeReverse = 1;

    public final int talonTimeout = 20;
}
