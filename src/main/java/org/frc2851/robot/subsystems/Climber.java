package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import org.frc2851.crevolib.utilities.Logger;
import org.frc2851.crevolib.utilities.TalonCommunicationErrorException;
import org.frc2851.crevolib.utilities.TalonSRXFactory;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

/**
 * Represents the climber subsystem
 */
public class Climber extends Subsystem
{
    public enum GorillaState
    {
        FORWARD(-0.75), REVERSE(0.5), NEUTRAL(0);

        final double power;

        GorillaState(double power)
        {
            this.power = power;
        }
    }

    public enum PogoState
    {
        FORWARD(-1.0), REVERSE(0.8), NEUTRAL(0);

        final double power;

        PogoState(double power)
        {
            this.power = power;
        }
    }

    private Constants mConst = Constants.getInstance();
    private Controller mController = Constants.driver;
    private GorillaState mCurrentGState = GorillaState.NEUTRAL;
    private PogoState mCurrentPState = PogoState.NEUTRAL;

    private DigitalInput mGForwardLimit, mGReverseLimit, mPForwardLimit, mPReverseLimit;
    private TalonSRX mGorillaMaster;
    private VictorSPX mPogoMaster;

    private static Climber mInstance = null;

    private Climber()
    {
        super("Climber");
    }

    /**
     * Returns the sole instance of the Climber class
     *
     * @return The instance of the Climber class
     */
    public static Climber getInstance()
    {
        if (mInstance == null) mInstance = new Climber();
        return mInstance;
    }

    /**
     * Initializes the controller, motors, and logging
     *
     * @return A boolean representing whether initialization has succeeded
     */
    @Override
    public boolean init()
    {
        mController.config(mConst.cl_gorillaForward, Button.ButtonMode.RAW);
        mController.config(mConst.cl_gorillaReverse, Button.ButtonMode.RAW);
        mController.config(mConst.cl_pogoForward, Button.ButtonMode.RAW);
        mController.config(mConst.cl_pogoReverse, Button.ButtonMode.RAW);

        try
        {
            mGorillaMaster = TalonSRXFactory.createDefaultTalonSRX(mConst.cl_gorillaMaster);
            mPogoMaster = new VictorSPX(mConst.cl_pogoMaster);

            mGorillaMaster.setNeutralMode(NeutralMode.Brake);
            mGorillaMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, mConst.talonTimeout);

            TalonSRXFactory.runTalonConfig(
                    () -> mPogoMaster.configNominalOutputForward(0),
                    () -> mPogoMaster.configNominalOutputReverse(0),
                    () -> mPogoMaster.configPeakOutputForward(1),
                    () -> mPogoMaster.configPeakOutputReverse(-1)
            );

        } catch (TalonCommunicationErrorException e)
        {
            log("Could not initialize motor, climber init failed! Port: " + e.getPortNumber(), Logger.LogLevel.ERROR);
            return false;
        }

        // Limit Switch Configuration
        if (mConst.cl_useGorillaLimit)
        {
            mGForwardLimit = new DigitalInput(mConst.cl_gorillaForwardLimit);
            mGReverseLimit = new DigitalInput(mConst.cl_gorillaReverseLimit);
        }

        if (mConst.cl_usePogoLimit)
        {
            mPForwardLimit = new DigitalInput(mConst.cl_pogoForwardLimit);
            mPReverseLimit = new DigitalInput(mConst.cl_pogoReverseLimit);
        }


        BadLog.createTopic("Gorilla/Master Gorilla Output", BadLog.UNITLESS, () -> mGorillaMaster.getMotorOutputPercent(), "hide", "join:Gorilla/Percent");
        BadLog.createTopic("Gorilla/Master Gorilla Voltage", "V", () -> mGorillaMaster.getMotorOutputVoltage(), "hide", "join:Gorilla/Voltage");
        BadLog.createTopic("Gorilla/Master Gorilla Current", "I", () -> mGorillaMaster.getOutputCurrent(), "hide", "join:Gorilla/Current");

        BadLog.createTopic("Pogo/Pogo Output", BadLog.UNITLESS, () -> mPogoMaster.getMotorOutputPercent(), "hide", "join:Pogo/Percent");
        BadLog.createTopic("Pogo/Pogo Voltage", "V", () -> mPogoMaster.getMotorOutputVoltage(), "hide", "join:Pogo/Voltage");

        return true;
    }

    /**
     * Resets the motors
     */
    private void reset()
    {
        mGorillaMaster.set(ControlMode.PercentOutput, 0);
        mPogoMaster.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Returns a command representing user control over the climber
     *
     * @return A command representing user control over the climber
     */
    @Override
    public Command getDefaultCommand()
    {
        return new Command()
        {
            @Override
            public String getName()
            {
                return "Teleop";
            }

            @Override
            public boolean isFinished()
            {
                return false;
            }

            @Override
            public boolean init()
            {
                reset();
                return true;
            }

            @Override
            public void update()
            {
                if (Robot.isRunning())
                {
                    GorillaState gState = GorillaState.NEUTRAL;
                    PogoState pState = PogoState.NEUTRAL;

                    // Poll Buttons and Checks limit switches
                    if (mController.get(mConst.cl_gorillaForward) && (!mConst.cl_useGorillaLimit || mGForwardLimit.get()))
                        gState = GorillaState.FORWARD;
                    else if (mController.get(mConst.cl_gorillaReverse) && (!mConst.cl_useGorillaLimit || mGReverseLimit.get()))
                        gState = GorillaState.REVERSE;

                    if (mController.get(mConst.cl_pogoForward) && (!mConst.cl_usePogoLimit || mPForwardLimit.get()))
                        pState = PogoState.FORWARD;
                    else if (mController.get(mConst.cl_pogoReverse) && (!mConst.cl_usePogoLimit || mPReverseLimit.get()))
                        pState = PogoState.REVERSE;

                    // Logging
                    if (gState != mCurrentGState) log("Updated Gorilla State: " + gState.name(), Logger.LogLevel.DEBUG);
                    if (pState != mCurrentPState) log("Updated Pogo State: " + pState.name(), Logger.LogLevel.DEBUG);
                    mCurrentGState = gState;
                    mCurrentPState = pState;

                    // Talon Set
                    mGorillaMaster.set(ControlMode.PercentOutput, gState.power);
                    mPogoMaster.set(ControlMode.PercentOutput, pState.power);
                }
            }

            @Override
            public void stop()
            {
                reset();
            }
        };
    }

    public GorillaState getGorillaState()
    {
        return mCurrentGState;
    }

    public PogoState getPogoState()
    {
        return mCurrentPState;
    }
}