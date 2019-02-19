package org.frc2851.robot;

import org.frc2851.crevolib.io.Axis;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.motion.PID;
import org.frc2851.robot.subsystems.DriveTrain;

/**
 * Stores constants used throughout the program
 */
public class Constants
{
    private static Constants mInstance;

    /**
     * Gets the sole instance of the Constants class
     *
     * @return The instance of the Constants class
     */
    public static Constants getInstance()
    {
        if (mInstance == null) mInstance = new Constants();
        return mInstance;
    }

    private Constants()
    {
    }

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

    // On the driver controller
    public final Button.ButtonID dtCurvatureToggle = Button.ButtonID.RIGHT_BUMPER;
    public final Button.ButtonID dtGearToggle = Button.ButtonID.LEFT_BUMPER;

    // Also has the two driver sticks all to itself for various purposes

    // Elevator
    public final int el_talon = 7;
//    public final int elevatorCanifier = 11;

    public final double el_rawMultiplier = 0.8;

    public final int el_maxVelocity = 0;
    public final int el_maxAcceleration = 0;
    public final int el_allowedClosedLoopError = 128;

    public final PID el_motionPID = new PID(0, 0, 0, 0);
    public final PID el_posPID = new PID(0, 0, 0, 0);

    // On the operator controller
    public final Button.ButtonID elevatorLowHatch = Button.ButtonID.START;
    public final Button.ButtonID elevatorMidHatch = Button.ButtonID.X;
    public final Button.ButtonID elevatorHighHatch = Button.ButtonID.Y;
    public final Button.ButtonID elevatorLowCargo = Button.ButtonID.B;
    public final Button.ButtonID elevatorMidCargo = Button.ButtonID.A;
    public final Button.ButtonID elevatorHighCargo = Button.ButtonID.SELECT;

    public final Axis.AxisID elevatorDirectControl = Axis.AxisID.RIGHT_Y;

    // Climber
    public final int gorillaMaster = 9;
    public final int gorillaSlave = 10;
    public final int screwMaster = 11;
    public final int gorillaLimitOut = 1;
    public final int gorillaLimitIn = 2;
    public final int screwLimitOut = 3;
    public final int screwLimitIn = 4;

    // On the driver controller
    public final Button.ButtonID climberGorillaButton = Button.ButtonID.A;
    public final Button.ButtonID climberScrewButton = Button.ButtonID.B;

    // Intake
    public final int intakeMaster = 0;
    public final int intakeForward = 0;
    public final int intakeReverse = 1;

    // On the operator controller
    public final Button.ButtonID intakeIntakeButton = Button.ButtonID.RIGHT_BUMPER;
    public final Button.ButtonID intakeOuttakeButton = Button.ButtonID.LEFT_BUMPER;

    // Hatcher
    public final int extendHatcherForward = 0;
    public final int extendHatcherReverse = 0;
    public final int actuateHatcherForward = 0;
    public final int actuateHatcherReverse = 0;

    // On the operator controller
    public final Button.ButtonID hatcherExtendButton = Button.ButtonID.Y;
    public final Button.ButtonID hatcherActuateButton = Button.ButtonID.X;

    // RollerClaw
    public final int rollerClawTalon = 1;
    public final int rollerClawLimitSwitch = 0;

    // On the operator controller
    public final Axis.AxisID rollerClawIntake = Axis.AxisID.RIGHT_TRIGGER;
    public final Axis.AxisID rollerClawOuttake = Axis.AxisID.LEFT_TRIGGER;

    public final int talonTimeout = 20;
}
