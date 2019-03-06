package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.motion.PID;
import org.frc2851.crevolib.utilities.TalonCommunicationErrorException;
import org.frc2851.crevolib.utilities.TalonSRXFactory;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.crevolib.utilities.CustomPreferences;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

public class Elevator extends Subsystem
{
    private enum GamePiece
    {
        HATCH, CARGO
    }

    public enum ElevatorControlMode
    {
        DIRECT(ControlMode.PercentOutput, -1),
        MOTION_MAGIC(ControlMode.MotionMagic, 0),
        POS_PID(ControlMode.Position, 1);

        private final ControlMode controlMode;
        private final int slotID;

        ElevatorControlMode(ControlMode controlMode, int slotID)
        {
            this.controlMode = controlMode;
            this.slotID = slotID;
        }

        ControlMode getMode()
        {
            return controlMode;
        }

        int getSlotID()
        {
            return slotID;
        }
    }

    public enum ElevatorPosition
    {
        LOW_HATCH(0, GamePiece.HATCH),
        MID_HATCH(9100, GamePiece.HATCH),
        HIGH_HATCH(18000, GamePiece.HATCH),
        LOW_CARGO(5800, GamePiece.CARGO),
        MID_CARGO(14500, GamePiece.CARGO),
        HIGH_CARGO(22500, GamePiece.CARGO),
        PLAYER_STATION(12250, GamePiece.CARGO),
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

    private Constants mConst = Constants.getInstance();

    private TalonSRX mTalon;
    private DigitalInput mForwardLimit, mReverseLimit;
    private Controller mController = Constants.operator;

    private ElevatorControlMode mControlMode = ElevatorControlMode.DIRECT;
    private ElevatorControlMode mClosedLoopStategy = ElevatorControlMode.MOTION_MAGIC;
    private ElevatorPosition mCurrentPosition = ElevatorPosition.LOW_HATCH;

    // Tuning
    CustomPreferences mTunePrefs = new CustomPreferences("ElevatorTuning");

    private final boolean TUNING = true;

    private static Elevator mInstance;

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
            mTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, mConst.talonTimeout);
            mTalon.setSensorPhase(true);
        } catch (TalonCommunicationErrorException e)
        {
            log("Could not initialize motor, elevator init failed! Port: " + e.getPortNumber(), Logger.LogLevel.ERROR);
            return false;
        }

        // Limit Switch Configuration
        mForwardLimit = new DigitalInput(mConst.el_forwardLimit);
        mReverseLimit = new DigitalInput(mConst.el_reverseLimit);

        configPreferences();
        configBadLog();
        configController(mController);
        zeroSensors();
        reset();

        return true;
    }

    private void configPreferences()
    {
        if (!mTunePrefs.containsKey("magP")) mTunePrefs.putDouble("magP", 0);
        if (!mTunePrefs.containsKey("magI")) mTunePrefs.putDouble("magI", 0);
        if (!mTunePrefs.containsKey("magD")) mTunePrefs.putDouble("magD", 0);
        if (!mTunePrefs.containsKey("magF")) mTunePrefs.putDouble("magF", 0);

        if (!mTunePrefs.containsKey("posP")) mTunePrefs.putDouble("posP", 0);
        if (!mTunePrefs.containsKey("posI")) mTunePrefs.putDouble("posI", 0);
        if (!mTunePrefs.containsKey("posD")) mTunePrefs.putDouble("posD", 0);
        if (!mTunePrefs.containsKey("posF")) mTunePrefs.putDouble("posF", 0);

        if (!mTunePrefs.containsKey("maxV")) mTunePrefs.putInt("maxV", 0);
        if (!mTunePrefs.containsKey("maxA")) mTunePrefs.putInt("maxA", 0);

        mTunePrefs.putInt("pos", 0);
        mTunePrefs.putInt("vel", 0);
        mTunePrefs.putInt("error", 0);
    }

    private void configBadLog()
    {
        BadLog.createTopicStr("Elevator/Control Mode", BadLog.UNITLESS, () -> mControlMode.name(), "hide");
        BadLog.createTopic("Elevator/Output Percent", BadLog.UNITLESS, () -> mTalon.getMotorOutputPercent(), "hide");
        BadLog.createTopic("Elevator/Output Voltage Master", "V", () -> mTalon.getBusVoltage(), "hide");
        BadLog.createTopic("Elevator/Output Current Master", "I", () -> mTalon.getOutputCurrent(), "hide");
        BadLog.createTopic("Elevator/Position", "Counts", () -> (double) mTalon.getSensorCollection().getQuadraturePosition(), "hide");
    }

    private void configController(Controller controller)
    {
        controller.config(mConst.el_midCargo, Button.ButtonMode.ON_PRESS);
        controller.config(mConst.el_lowCargo, Button.ButtonMode.ON_PRESS);
        controller.config(mConst.el_midHatch, Button.ButtonMode.ON_PRESS);
        controller.config(mConst.el_highHatch, Button.ButtonMode.ON_PRESS);
        controller.config(mConst.el_lowHatch, Button.ButtonMode.ON_PRESS);
        controller.config(mConst.el_highCargo, Button.ButtonMode.ON_PRESS);

        // checks limit switches, applies deadband, and applies multiplier
        controller.config(mConst.el_rawControl, (x) -> {
            x = (!mReverseLimit.get() && x > 0) ? 0 : x;
            return -(x * mConst.el_rawMultiplier);
        });
//        controller.setDeadband(mConst.el_rawControl, 0.15, false);
    }

    private void zeroSensors()
    {
        TalonSRXFactory.runTalonConfig(() -> mTalon.setSelectedSensorPosition(0, 0, mConst.talonTimeout));
    }

    public void reset()
    {
        mTalon.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public Command getDefaultCommand()
    {
        return new Command()
        {
            ElevatorPosition desiredPosition = null;

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

            @Override
            public void update()
            {
                if (Robot.isRunning())
                {
                    double output = mController.get(mConst.el_rawControl);
                    getDesiredPosition();
                    boolean updatePos = mCurrentPosition != desiredPosition;

                    mTunePrefs.putInt("pos", mTalon.getSelectedSensorPosition(0));

                    if (updatePos)
                    {
                        mCurrentPosition = desiredPosition;
                        setCommmandGroup(setPositionMotionProfiling(desiredPosition));
                    }

                    if (output != 0)
                    {
                        stopAuxilaryCommand();
                        mTalon.set(ControlMode.PercentOutput, output);
                    } else if (!getAuxilaryCommandActivity())
                    {
                        mTalon.set(ControlMode.PercentOutput, 0.1);
                    }

                    mTunePrefs.putInt("pos", mTalon.getSelectedSensorPosition(0));
                    mTunePrefs.putInt("vel", mTalon.getSelectedSensorVelocity(0));
                }
            }

            @Override
            public void stop()
            {
            }

            void getDesiredPosition()
            {
                if (mController.get(mConst.el_lowHatch))
                    desiredPosition = ElevatorPosition.LOW_HATCH;
                else if (mController.get(mConst.el_midHatch))
                    desiredPosition = ElevatorPosition.MID_HATCH;
                else if (mController.get(mConst.el_highHatch))
                    desiredPosition = ElevatorPosition.HIGH_HATCH;
                else if (mController.get(mConst.el_lowCargo))
                    desiredPosition = ElevatorPosition.LOW_CARGO;
                else if (mController.get(mConst.el_midCargo))
                    desiredPosition = ElevatorPosition.MID_CARGO;
                else if (mController.get(mConst.el_highCargo))
                    desiredPosition = ElevatorPosition.HIGH_CARGO;
                else if (mController.get(mConst.el_playerStation))
                    desiredPosition = ElevatorPosition.PLAYER_STATION;
            }
        };
    }

    public Command setPositionMotionProfiling(ElevatorPosition pos)
    {
        return new Command()
        {
            boolean hitLimit = false;

            @Override
            public String getName()
            {
                return "SetPositionMP[" + pos.name() + "]";
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
                    return Math.abs(mTalon.getClosedLoopError(0)) < mConst.el_allowableHatchError;
                else
                    return Math.abs(mTalon.getClosedLoopError(0)) < mConst.el_allowableCargoError;
            }

            @Override
            public boolean init()
            {
                double p = (TUNING) ? mTunePrefs.getDouble("magP", 0) : mConst.el_motionPID.getP();
                double i = (TUNING) ? mTunePrefs.getDouble("magI", 0) : mConst.el_motionPID.getI();
                double d = (TUNING) ? mTunePrefs.getDouble("magD", 0) : mConst.el_motionPID.getD();
                double f = (TUNING) ? mTunePrefs.getDouble("magF", 0) : mConst.el_motionPID.getF();
                int maxV = (TUNING) ? mTunePrefs.getInt("maxV", 0) : mConst.el_maxVelocity;
                int maxA = (TUNING) ? mTunePrefs.getInt("maxA", 0) : mConst.el_maxAcceleration;

                TalonSRXFactory.configurePIDF(mTalon, ElevatorControlMode.MOTION_MAGIC.getSlotID(), new PID(p, i, d, f));
                TalonSRXFactory.runTalonConfig(
                        () -> mTalon.configMotionCruiseVelocity(maxV, mConst.talonTimeout),
                        () -> mTalon.configMotionAcceleration(maxA, mConst.talonTimeout)
                );

                mTalon.selectProfileSlot(ElevatorControlMode.MOTION_MAGIC.getSlotID(), 0);
                mTalon.set(ControlMode.MotionMagic, pos.getPos());
                return true;
            }

            @Override
            public void update()
            {
                hitLimit = pos.getPos() < mTalon.getSelectedSensorPosition(0) && !mReverseLimit.get();
                mTunePrefs.putInt("error", mTalon.getClosedLoopError(0));
            }

            @Override
            public void stop()
            {
                mTalon.set(ControlMode.PercentOutput, 0);
            }
        };
    }

    public Command setPosition(int counts, String posName)
    {
        return new Command()
        {
            boolean isFinished = false;
            private final double upMult = 1, downMult = 0.8;

            @Override
            public String getName()
            {
                return "SetPosition[P: " + posName + ", C: " + counts + "]";
            }

            @Override
            public boolean isFinished()
            {
                return isFinished;
            }

            @Override
            public boolean init()
            {
                if (counts == -1)
                {
                    log("Could not read setpoint!", Logger.LogLevel.ERROR);
                    return false;
                }
                return true;
            }

            @Override
            public void update()
            {
                int error = counts - mTalon.getSelectedSensorPosition(0);

                double output = Math.copySign(0.8, error);

                mTalon.set(ControlMode.PercentOutput, output);

                if (TUNING)
                {
                    mTunePrefs.putInt("pos", mTalon.getSelectedSensorPosition(0));
                    mTunePrefs.putInt("vel", mTalon.getSelectedSensorVelocity(0));
                    mTunePrefs.putInt("error", mTalon.getClosedLoopError(0));
                }

                isFinished = error < 500;
            }

            @Override
            public void stop()
            {
                mTalon.set(ControlMode.PercentOutput, 0.05);
            }
        };
    }

    public Command setPosition(ElevatorPosition pos)
    {
        return setPosition(pos.getPos(), pos.name());
    }

    public Command setPosition(int counts)
    {
        return setPosition(counts, "null");
    }

    public Command moveForTime(double time, double power)
    {
        return new Command()
        {
            double startTime;
            boolean limitTriggered = false;

            @Override
            public String getName()
            {
                return "MoveForTime[T: " + time + ", P: " + power + "]";
            }

            @Override
            public boolean isFinished()
            {
                return (Timer.getFPGATimestamp() - startTime) > time || limitTriggered;
            }

            @Override
            public boolean init()
            {
                startTime = Timer.getFPGATimestamp();
                return true;
            }

            @Override
            public void update()
            {
                limitTriggered = !mReverseLimit.get() && power < 0;
                mTalon.set(ControlMode.PercentOutput, power);
            }

            @Override
            public void stop()
            {
                mTalon.set(ControlMode.PercentOutput, 0);
            }
        };
    }
}
