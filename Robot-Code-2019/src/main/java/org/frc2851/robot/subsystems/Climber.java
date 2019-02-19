package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.drivers.TalonCommunicationErrorException;
import org.frc2851.crevolib.drivers.TalonSRXFactory;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;
// button id references for ME to use.(you can use them too if you want)
/*A(1), B(2), X(3), Y(4), START(8), SELECT(7), LEFT_BUMPER(5), RIGHT_BUMPER(6),
        LEFT_JOYSTICK(9), RIGHT_JOYSTICK(10);
        LEFT_X(0), LEFT_Y(1), RIGHT_X(4), RIGHT_Y(5), LEFT_TRIGGER(2), RIGHT_TRIGGER(3);*/

/**
 * Represents the climber subsystem
 */
public class Climber extends Subsystem
{
    public enum GorillaState
    {
        FORWARD(0.5), REVERSE(-0.5), NEUTRAL(0);

        final double power;

        GorillaState(double power)
        {
            this.power = power;
        }
    }

    public enum PogoState
    {
        FORWARD(0.1), REVERSE(-0.1), NEUTRAL(0);

        final double power;

        PogoState(double power)
        {
            this.power = power;
        }
    }

    private Constants mConst = Constants.getInstance();
    private Controller mController = Robot.driver;
    private static Climber mInstance = null;

    private DigitalInput mGForwardLimit, mGReverseLimit, mPForwardLimit, mPReverseLimit;

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
     * Initializes the Climber class with the name "Climber"
     */
    private Climber()
    {
        super("Climber");
    }

    private WPI_TalonSRX mGorillaMaster, mGorillaSlave /*mPogoMaster*/;
    private VictorSPX mPogoMaster;

    /**
     * Initializes the controller, motors, and logging
     *
     * @return A boolean representing whether initialization has succeeded
     */
    @Override
    public boolean init()
    {
        mController.config(mConst.climberGorillaButton, Button.ButtonMode.RAW);
        mController.config(mConst.climberScrewButton, Button.ButtonMode.RAW);
        mController.config(Button.ButtonID.X, Button.ButtonMode.RAW);
        mController.config(Button.ButtonID.Y, Button.ButtonMode.RAW);

        try
        {
            mGorillaMaster = TalonSRXFactory.createDefaultWPI_TalonSRX(mConst.cl_gorillaMaster);
            mGorillaSlave = TalonSRXFactory.createPermanentSlaveWPI_TalonSRX(mConst.cl_gorillaSlave, mGorillaMaster);
            mPogoMaster = new VictorSPX(mConst.cl_pogoMaster); //TalonSRXFactory.createDefaultWPI_TalonSRX(mConst.screwMaster);
        } catch (TalonCommunicationErrorException e)
        {
            log("Could not initialize motor, climber init failed! Port: " + e.getPortNumber(), Logger.LogLevel.ERROR);
            return false;
        }

        // Limit Switch Configuration

        if (mConst.cl_useTalonLimit)
        {
            boolean setsSucceeded = true;
            int tries = 0;
            final int maxTries = 5;
            do
            {
                setsSucceeded &= mGorillaMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                        LimitSwitchNormal.NormallyOpen) == ErrorCode.OK;

                setsSucceeded &= mGorillaMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                        LimitSwitchNormal.NormallyOpen) == ErrorCode.OK;

                setsSucceeded &= mPogoMaster.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX,
                        LimitSwitchNormal.NormallyOpen,
                        mConst.cl_pogoLimitRemoteID,
                        mConst.talonTimeout) == ErrorCode.OK;

                setsSucceeded &= mPogoMaster.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX,
                        LimitSwitchNormal.NormallyOpen,
                        mConst.cl_pogoLimitRemoteID,
                        mConst.talonTimeout) == ErrorCode.OK;
            } while (!setsSucceeded && tries++ < maxTries);
            if (!setsSucceeded)
            {
                log("Could not initialize limit switches", Logger.LogLevel.ERROR);
                return false;
            }
        } else
        {
            mGForwardLimit = new DigitalInput(mConst.gorillaLimitOut);
            mGReverseLimit = new DigitalInput(mConst.gorillaLimitIn);
            mPForwardLimit = new DigitalInput(mConst.screwLimitOut);
            mPReverseLimit = new DigitalInput(mConst.screwLimitIn);
        }

        BadLog.createTopic("Climber/Master Gorilla Output", BadLog.UNITLESS, () -> mGorillaMaster.getMotorOutputPercent(), "hide", "join:Climber/Percent Output");
        BadLog.createTopic("Climber/Slave Gorilla Output", BadLog.UNITLESS, () -> mGorillaSlave.getMotorOutputPercent(), "hide", "join:Climber/Percent Output");
        BadLog.createTopic("Climber/Screw Output", BadLog.UNITLESS, () -> mPogoMaster.getMotorOutputPercent(), "hide", "join:Climber/Percent Output");
        BadLog.createTopic("Climber/Master Gorilla Voltage", "Voltage:", () -> mGorillaMaster.getBusVoltage(), "hide", "join:Climber/Voltage Outputs");
        BadLog.createTopic("Climber/Slave Gorilla Voltage", "Voltage:", () -> mGorillaSlave.getBusVoltage(), "hide", "join:Climber/Voltage Outputs");
        BadLog.createTopic("Climber/Screw Voltage", "Voltage:", () -> mPogoMaster.getBusVoltage(), "hide", "join:Climber/Percent Output");
        BadLog.createTopic("Climber/Master Gorilla Current", "Amperes:", () -> mGorillaMaster.getOutputCurrent(), "hide", "join:Climber/Current Outputs");
        BadLog.createTopic("Climber/Slave Gorilla Current", "Amperes:", () -> mGorillaSlave.getOutputCurrent(), "hide", "join:Climber/Current Outputs");
        return true;
    }

    /**
     * Resets the motors
     */
    private void reset()
    {
        mGorillaMaster.set(ControlMode.PercentOutput, 0);
        mPogoMaster.set(ControlMode.PercentOutput, 0);
//        log("All motors zeroed", Logger.LogLevel.DEBUG);
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
            GorillaState lastGState = GorillaState.NEUTRAL;
            PogoState lastPState = PogoState.NEUTRAL;

            boolean trippedCurrentDraw = false;

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
                    if (mController.get(Button.ButtonID.A) && (mConst.cl_useTalonLimit || !mGForwardLimit.get()))
                        gState = GorillaState.FORWARD;
                    else if (mController.get(Button.ButtonID.B) && (mConst.cl_useTalonLimit || !mGReverseLimit.get()))
                        gState = GorillaState.REVERSE;

                    if (mController.get(Button.ButtonID.X) && (mConst.cl_useTalonLimit || !mPForwardLimit.get()))
                        pState = PogoState.FORWARD;
                    else if (mController.get(Button.ButtonID.Y) && (mConst.cl_useTalonLimit || !mPReverseLimit.get()))
                        pState = PogoState.REVERSE;

                    // Logging
                    if (gState != lastGState) log("Updated Gorilla State: " + gState.name(), Logger.LogLevel.DEBUG);
                    if (pState != lastPState) log("Updated Pogo State: " + pState.name(), Logger.LogLevel.DEBUG);
                    lastGState = gState;
                    lastPState = pState;

                    // TODO: Tune reverse threshold voltage
//                    if (mGorillaMaster.getOutputCurrent() > mConst.cl_reverseThresholdCurrent && gState == GorillaState.REVERSE)
//                    {
//                        log("Current threshold reached, disabling gorilla arm", Logger.LogLevel.ERROR);
//                        trippedCurrentDraw = true;
//                    }

                    // Talon Set
                    if (!trippedCurrentDraw) mGorillaMaster.set(ControlMode.PercentOutput, gState.power);
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
}