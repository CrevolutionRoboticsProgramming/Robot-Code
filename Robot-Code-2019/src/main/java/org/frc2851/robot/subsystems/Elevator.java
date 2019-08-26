package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.motion.PID;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.crevolib.utilities.CustomPreferences;
import org.frc2851.crevolib.utilities.Logger;
import org.frc2851.crevolib.utilities.TalonCommunicationErrorException;
import org.frc2851.crevolib.utilities.TalonSRXFactory;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

/*
 * TODO: Determine motor direction and set encoder phase accordingly
 */

/**
 * Represents the elevator subsystem
 */
public class Elevator extends Subsystem
{
    private enum ElevatorControlMode { OPEN_LOOP, MOTION_MAGIC }

    // Singleton Definitions
    private static Elevator mInstance;
    private ElevatorControlMode mControlMode = ElevatorControlMode.OPEN_LOOP;
    private final boolean kTuning = false;
    // Tuning
    CustomPreferences mTunePrefs = new CustomPreferences("ElevatorTuning");
    private Constants mConst = Constants.getInstance();
    private TalonSRX mTalon;
    private DigitalInput mLimitSwitch;
    private Controller mController = Constants.operator;
    private ElevatorPosition mCurrentPosition = ElevatorPosition.LOW_HATCH;

    private boolean mSetPositionRunning = false;

    private Elevator()
    {
        super("Elevator");
    }

    public static Elevator getInstance()
    {
        if (mInstance == null) mInstance = new Elevator();
        return mInstance;
    }

    @Override
    protected boolean init()
    {
        // Talon Configuration
        try
        {
            mTalon = TalonSRXFactory.createDefaultMasterTalonSRX(mConst.el_talon);
            mTalon.setNeutralMode(NeutralMode.Brake);

            TalonSRXFactory.runTalonConfig(
                    () -> mTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, mConst.talonTimeout),
                    () -> mTalon.configReverseSoftLimitThreshold(0, mConst.talonTimeout)
            );

            mTalon.setSensorPhase(true);
        } catch (TalonCommunicationErrorException e)
        {
            log("Could not initialize motor, elevator init failed! Port: " + e.getPortNumber(), Logger.LogLevel.ERROR);
            return false;
        }

        // Limit Switch Configuration
        mLimitSwitch = new DigitalInput(mConst.el_reverseLimit);

        configPreferences();
        configBadLog();
        configController(mController);
        zeroSensors();
        reset();

        return true;
    }

    private void configPreferences()
    {
        if (!mTunePrefs.containsKey("kP")) mTunePrefs.putDouble("kP", 0);
        if (!mTunePrefs.containsKey("kI")) mTunePrefs.putDouble("kI", 0);
        if (!mTunePrefs.containsKey("kD")) mTunePrefs.putDouble("kD", 0);
        if (!mTunePrefs.containsKey("kF")) mTunePrefs.putDouble("kF", 0);

        if (!mTunePrefs.containsKey("maxV")) mTunePrefs.putInt("maxV", 0);
        if (!mTunePrefs.containsKey("maxA")) mTunePrefs.putInt("maxA", 0);

        mTunePrefs.putInt("pos", 0);
        mTunePrefs.putInt("vel", 0);
        mTunePrefs.putInt("error", 0);
    }

    private void configBadLog()
    {
        BadLog.createTopic("Elevator/Output Percent", BadLog.UNITLESS, () -> mTalon.getMotorOutputPercent(), "hide");
        BadLog.createTopic("Elevator/Output Voltage Master", "V", () -> mTalon.getBusVoltage(), "hide");
        BadLog.createTopic("Elevator/Output Current Master", "I", () -> mTalon.getOutputCurrent(), "hide");
        BadLog.createTopic("Elevator/Position", "Counts", () -> (double) mTalon.getSelectedSensorPosition(0), "hide");
        BadLog.createTopic("Elevator/Velocity", "Counts/100ms", () -> (double) mTalon.getSelectedSensorVelocity(0));
    }

    private void configController(Controller controller)
    {
        controller.config(mConst.el_low, Button.ButtonMode.RAW);
        controller.config(mConst.el_mid, Button.ButtonMode.RAW);
        controller.config(mConst.el_high, Button.ButtonMode.RAW);
        controller.config(mConst.el_playerStation, Button.ButtonMode.RAW);
        controller.config(mConst.el_toggle, Button.ButtonMode.RAW);
        controller.config(mConst.el_rawControl);

        controller.config(Button.ButtonID.D_DOWN, Button.ButtonMode.TOGGLE);
    }

    private void zeroSensors()
    {
        TalonSRXFactory.runTalonConfig(() -> mTalon.setSelectedSensorPosition(0, 0, mConst.talonTimeout));
    }

    public void reset()
    {
        mTunePrefs.putInt("error", 0);
        mTalon.set(ControlMode.PercentOutput, 0);
        mCurrentPosition = ElevatorPosition.LOW_HATCH;
    }

    @Override
    public Command getDefaultCommand()
    {
        return new Command()
        {
            ElevatorPosition desiredPosition = ElevatorPosition.LOW_HATCH;

            @Override
            public String getName()
            {
                return "Default";
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

            double applyDeadband(double in, double band)
            {
                return (Math.abs(in) < band) ? 0 : in;
            }

            @Override
            public void update()
            {
                if (Robot.isRunning())
                {
                    double output = -applyDeadband(mController.get(mConst.el_rawControl), 0.15);
                    getDesiredPosition();
                    boolean updatePos = mCurrentPosition != desiredPosition;

                    if (!mLimitSwitch.get() && mCurrentPosition != ElevatorPosition.LOW_HATCH)
                    {
                        mCurrentPosition = ElevatorPosition.LOW_HATCH;
                        zeroSensors();
                        if (output < 0) output = 0;
                    }

                    if (output != 0)
                    {
                        stopAuxiliaryCommand();
                        mControlMode = ElevatorControlMode.OPEN_LOOP;
                        mTalon.set(ControlMode.PercentOutput, output);
                        mCurrentPosition = ElevatorPosition.UNKNOWN;
                    } else if (updatePos && desiredPosition != null)
                    {
                        mControlMode = ElevatorControlMode.MOTION_MAGIC;
                        setCommmandGroup(setPositionMotionMagic(desiredPosition));
                        mCurrentPosition = desiredPosition;
                    } else if (mControlMode == ElevatorControlMode.OPEN_LOOP)
                    {
                        mTalon.set(ControlMode.PercentOutput, mConst.el_holdPositionPower);
                    }
                }
            }

            @Override
            public void stop()
            {
                mTalon.set(ControlMode.PercentOutput, 0);
            }

            void getDesiredPosition()
            {
                if (!mController.get(Button.ButtonID.RIGHT_TRIGGER))
                {
                    if (mController.get(mConst.el_low))
                        desiredPosition = ElevatorPosition.LOW_HATCH;
                    else if (mController.get(mConst.el_mid))
                        desiredPosition = ElevatorPosition.MID_HATCH;
                    else if (mController.get(mConst.el_high))
                        desiredPosition = ElevatorPosition.HIGH_HATCH;
                    else if (mController.get(mConst.el_playerStation))
                        desiredPosition = ElevatorPosition.PLAYER_STATION;
                    else
                        desiredPosition = null;
                } else
                {
                    if (mController.get(mConst.el_low))
                        desiredPosition = ElevatorPosition.LOW_CARGO;
                    else if (mController.get(mConst.el_mid))
                        desiredPosition = ElevatorPosition.MID_CARGO;
                    else if (mController.get(mConst.el_high))
                        desiredPosition = ElevatorPosition.HIGH_CARGO;
                    else if (mController.get(mConst.el_playerStation))
                        desiredPosition = ElevatorPosition.PLAYER_STATION;
                    else
                        desiredPosition = null;
                }
            }
        };
    }

    /**
     * Returns a Command that sets the position of the elevator via motion magic
     *
     * @param pos The position to move the elevator to
     * @return A Command that sets the position of the elevator via PID and/or motion magic
     */
    public Command setPositionMotionMagic(ElevatorPosition pos)
    {
        return new Command()
        {
            boolean hitLimit = false;

            @Override
            public String getName()
            {
                return "SetPositionMP[" + "N: " + pos.name() + ", C:" + pos.getPos() + "]";
            }

            @Override
            public boolean isFinished()
            {
                if (hitLimit)
                {
                    log("Could not move successfully, hit limit", Logger.LogLevel.WARNING);
                    return true;
                }

                if (pos.piece == GamePiece.HATCH)
                    return (Math.abs(pos.getPos() - mTalon.getSelectedSensorPosition(0)) < mConst.el_allowableHatchError);
                else
                    return (Math.abs(pos.getPos() - mTalon.getSelectedSensorPosition(0)) < mConst.el_allowableCargoError);
            }

            @Override
            public boolean init()
            {
                double p = (kTuning) ? mTunePrefs.getDouble("kP", 0) : mConst.el_pid.getP();
                double i = (kTuning) ? mTunePrefs.getDouble("kI", 0) : mConst.el_pid.getI();
                double d = (kTuning) ? mTunePrefs.getDouble("kD", 0) : mConst.el_pid.getD();
                double f = mConst.el_pid.getF();
                int maxV = (kTuning) ? mTunePrefs.getInt("maxV", 0) :
                        ((pos.getPos() < mTalon.getSelectedSensorPosition(0)) ? mConst.el_maxVelocityDown : mConst.el_maxVelocityUp);
                int maxA = (kTuning) ? mTunePrefs.getInt("maxA", 0) :
                        ((pos.getPos() < mTalon.getSelectedSensorPosition(0)) ? mConst.el_maxAccelerationDown : mConst.el_maxAccelerationUp);

                PID pid = new PID(p, i, d, f);
                log("MP_INFO[Name: " + pos.name() + ", PID: " + pid.toString() + ", MaxVel: " + maxV + ", MaxAcc: " + maxA + "]", Logger.LogLevel.DEBUG);

                mTalon.selectProfileSlot(0, 0);
                TalonSRXFactory.configurePIDF(mTalon, 0, pid);
                TalonSRXFactory.runTalonConfig(
                        () -> mTalon.configMotionCruiseVelocity(maxV, mConst.talonTimeout),
                        () -> mTalon.configMotionAcceleration(maxA, mConst.talonTimeout)
                );

                mSetPositionRunning = true;
                return true;
            }

            @Override
            public void update()
            {
                int position = mTalon.getSelectedSensorPosition(0);
                hitLimit = pos.getPos() < position && !mLimitSwitch.get();
                mTunePrefs.putInt("error", pos.getPos() - position);

                mTalon.set(ControlMode.MotionMagic, pos.getPos());
            }

            @Override
            public void stop()
            {
                if (hitLimit) zeroSensors();
                mTalon.set(ControlMode.PercentOutput, 0);
                mControlMode = ElevatorControlMode.OPEN_LOOP;
            }
        };
    }

    private enum GamePiece
    {
        HATCH, CARGO
    }

    public enum ElevatorPosition
    {
        LOW_HATCH(0, GamePiece.HATCH),
        MID_HATCH(9100, GamePiece.HATCH),
        HIGH_HATCH(18000, GamePiece.HATCH),
        LOW_CARGO(5800, GamePiece.CARGO),
        MID_CARGO(14500, GamePiece.CARGO),
        HIGH_CARGO(22000, GamePiece.CARGO),
        PLAYER_STATION(11200, GamePiece.CARGO),
        UNKNOWN(0, null);

        private final int pos;
        private final GamePiece piece;
        private Constants c = Constants.getInstance();

        ElevatorPosition(int pos, GamePiece piece)
        {
            this.pos = pos;
            this.piece = piece;
        }

        public int getPos()
        {
            return pos;
        }

        public int getAllowableError()
        {
            if (piece == GamePiece.HATCH) return c.el_allowableHatchError;
            else if (piece == GamePiece.CARGO) return c.el_allowableCargoError;
            else return 0;
        }

        @Override
        public String toString()
        {
            return "[" + this.name() + ", " + pos + "]";
        }
    }
}