package org.frc2851.robot;

import org.frc2851.robot.subsystems.DriveTrain;

/**
 * creates a class to Store and use Constants
 */
public class Constants
{
    private static Constants mInstance;

    /**
     * Gets the sole instance of the Constants class
     * @return The instance of the Constants class
     */
    public static Constants getInstance() {
        if (mInstance == null) mInstance = new Constants();
        return mInstance;
    }

    private Constants() { }

    public final int magEncCPR = 4096;
    public final int pcmID = 0;

    public boolean singleControllerMode = false;

    // Drivetrain
    public final int dt_leftMaster = 1;
    public final int dt_leftSlaveA = 2;
    public final int dt_leftSlaveB = 3;
    public final int dt_rightMaster = 4;
    public final int dt_rightSlaveA = 5;
    public final int dt_rightSlaveB = 6;

    public final int dt_pigeonID = 42;
    public final int dt_pigeonRemoteOrdinalLeft = 0;
    public final int dt_pigeonRemoteOrdinalRight = 0;
    public final boolean dt_usePigeon = false;

    public final DriveTrain.DriveGear dt_defaultDriveGear = DriveTrain.DriveGear.HIGH;
    public final int dt_shifterForwardChannel = 7;
    public final int dt_shifterReverseChannel = 8;

    public final double dt_width = 2.5;
    public final double dt_wheelDiameter = 1.0d / 3.0d;

    public final int dt_drivePidSlot = 0;
    public final int dt_gyroPidSlot = 1;
    public final int dt_encTurnPidSlot = 2;

    // Units (Average Counts per 100ms)
    // TODO: Experimentally determine values
    public final int dt_motionMagicMaxAcceleration = 15000;
    public final int dt_motionMagicMaxVelocity = 3000;

    public final double dt_peakOut = 1;
    public final double dt_nominalOut = 0;

    // Elevator
    public final int elevatorMaster = 7;
    public final int elevatorSlave = 8;
    public final int elevatorCanifier = 11;

    public final double elevatorRawMultiplier = 0.8;

    public final int elevatorMaximumVelocity = 0;
    public final int elevatorMaximumAcceleration = 0;
    public final int elevatorAllowedClosedLoopError = 128;

    public final double elevatorMotionP = 0;
    public final double elevatorMotionI = 0;
    public final double elevatorMotionD = 0;
    public final double elevatorMotionF = 0;

    public final double elevatorPosP = 0;
    public final double elevatorPosI = 0;
    public final double elevatorPosD = 0;
    public final double elevatorPosF = 0;

    // Climber
    public final int gorillaMaster = 9;
    public final int gorillaSlave = 10;
    public final int screwMaster = 11;

    // Intake
    public final int intakeMaster = 0;
    public final int intakeForward = 0;
    public final int intakeReverse = 1;

    // Roller Claw
    public final int rollerClawTalon=0;

    // Hatcher
    public final int extendHatcherForward = 0;
    public final int extendHatcherReverse = 0;
    public final int actuateHatcherForward = 0;
    public final int actuateHatcherReverse = 0;

    public final int talonTimeout = 20;
}
