package org.frc2851.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import org.frc2851.crevolib.io.Axis;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
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

    public static Controller driver = new Controller(0);
    public static Controller operator = new Controller(1);

    /************ Motor Controllers and Others ************/
    // CAN Bus
    public final int pdp = 0;
    public final int dt_leftMaster = 1;
    public final int dt_leftSlaveA = 2;
    public final int dt_leftSlaveB = 3;
    public final int dt_rightMaster = 4;
    public final int dt_rightSlaveA = 5;
    public final int dt_rightSlaveB = 6;
    public final int el_talon = 7;
    public final int cl_gorillaMaster = 8;
    public final int cl_gorillaSlave = 9;
    public final int cl_pogoMaster = 10;
    public final int in_talon = 11;
    public final int rc_talon = 12;
    public final int pcm = 13;

    // Pneumatics
    public final int in_solenoidForward = 6;
    public final int in_solenoidReverse = 7;
    public final int ht_extendForward = 0;
    public final int ht_extendReverse = 1;
    public final int ht_actuateForward = 2;
    public final int ht_actuateReverse = 3;
    public final int dt_shifterForwardChannel = 4;
    public final int dt_shifterReverseChannel = 5;

    // DIO
    public final int rc_limitSwitch = 0; // Normally Open; Closed when ball in RC
    public final int cl_gorillaForwardLimit = 1;
    public final int cl_gorillaReverseLimit = 2;
    public final int cl_pogoForwardLimit = 3;
    public final int cl_pogoReverseLimit = 4;
    public final int el_reverseLimit = 6; // Closed at min height

    /************ Controller Buttons and Axis ************/
    // Drivetrain - Driver Controller
    public final Button.ButtonID dt_curvatureToggle = Button.ButtonID.RIGHT_BUMPER;
    public final Button.ButtonID dt_gearToggle = Button.ButtonID.LEFT_BUMPER;

    // Elevator - Operator Controller
    public final Button.ButtonID el_playerStation = Button.ButtonID.D_UP;
    public final Button.ButtonID el_low = Button.ButtonID.A;
    public final Button.ButtonID el_mid = Button.ButtonID.X;
    public final Button.ButtonID el_high = Button.ButtonID.Y;
    public final Button.ButtonID el_toggle = Button.ButtonID.RIGHT_TRIGGER;
    public final Axis.AxisID el_rawControl = Axis.AxisID.RIGHT_Y;

    // Gorilla Arm / Pogo - Driver Controller
    public final Button.ButtonID cl_gorillaForward = Button.ButtonID.A;
    public final Button.ButtonID cl_gorillaReverse = Button.ButtonID.B;
    public final Button.ButtonID cl_pogoForward = Button.ButtonID.X;
    public final Button.ButtonID cl_pogoReverse = Button.ButtonID.Y;

    // Hatcher - Operator Controller
    public final Button.ButtonID ht_extend = Button.ButtonID.LEFT_BUMPER;
    public final Button.ButtonID ht_actuate = Button.ButtonID.RIGHT_BUMPER;

    // Intake - Driver Controller
    public final Button.ButtonID in_extend = Button.ButtonID.RIGHT_BUMPER;
    public final Button.ButtonID in_intake = Button.ButtonID.RIGHT_TRIGGER;
    public final Button.ButtonID in_outake = Button.ButtonID.LEFT_TRIGGER;

    // Roller Claw - Driver Controller
    public final Button.ButtonID rc_intake = Button.ButtonID.RIGHT_TRIGGER;
    public final Button.ButtonID rc_outtake = Button.ButtonID.LEFT_TRIGGER;
    public final Button.ButtonID rc_hold = Button.ButtonID.D_UP;

    /************ Drivetrain ************/
    // Pigeon
    public final int dt_pigeonID = 42;
    public final int dt_pigeonRemoteOrdinalLeft = 0;
    public final int dt_pigeonRemoteOrdinalRight = 0;
    public final boolean dt_usePigeon = false;

    // Shifters
    public final DriveTrain.DriveGear dt_defaultDriveGear = DriveTrain.DriveGear.HIGH;

    // Physical Dimensions
    public final double dt_width = 2.5;
    public final double dt_wheelDiameter = 1.0d / 3.0d;

    // Closed Loop Constants
    public final int dt_drivePidSlot = 0;
    public final int dt_gyroPidSlot = 1;
    public final int dt_encTurnPidSlot = 2;
    public final int dt_motionMagicMaxAcceleration = 15000;
    public final int dt_motionMagicMaxVelocity = 3000;

    // Defaults
    public final double dt_peakOut = 1;
    public final double dt_nominalOut = 0;
    public final NeutralMode dt_defaultNeutralMode = NeutralMode.Brake;

    /************ Elevator ************/
    public final double el_rawMultiplier = 1;
    public final double el_holdPositionPower = 0.1;

    public final int el_allowableHatchError = 250;
    public final int el_allowableCargoError = 100;

    public final int el_maxVelocityDown = 1500;
    public final int el_maxAccelerationDown = 1500;
    public final int el_maxVelocityUp = 1500;
    public final int el_maxAccelerationUp = 1500;
    public final PID el_pid = new PID(0.3, 0, 0, 1);

    /************ Climber ************/
    public final boolean cl_usePogoLimit = false;
    public final boolean cl_useGorillaLimit = true;

    /************ Roller Claw ************/
    public final boolean rc_useLimit = false;

    public final int talonTimeout = 20;
}