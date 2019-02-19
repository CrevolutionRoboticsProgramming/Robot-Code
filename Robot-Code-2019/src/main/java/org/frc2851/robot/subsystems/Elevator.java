package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.drivers.TalonCommunicationErrorException;
import org.frc2851.crevolib.drivers.TalonSRXFactory;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.crevolib.utilities.CustomPreferences;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

/**
 * controls the Elevator Motors
 */

/*
 * TODO: Determine motor direction and set encoder phase accordingly
 */
public class Elevator extends Subsystem
{
    public enum ElevatorControlMode
    {
        DIRECT(ControlMode.PercentOutput, -1),
        MOTION_MAGIC(ControlMode.MotionMagic, 0),
        POS_PID(ControlMode.Position, 1),
        CURRENT(ControlMode.Current, 2);

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
        LOW_HATCH(0), MID_HATCH(0), HIGH_HATCH(0),
        LOW_CARGO(0), MID_CARGO(0), HIGH_CARGO(0);

        private final int pos;

        ElevatorPosition(int pos)
        {
            this.pos = pos;
        }

        public int getPos()
        {
            return pos;
        }

        @Override
        public String toString()
        {
            return "[" + this.name() + ", " + pos + "]";
        }
    }

    private Constants mConst = Constants.getInstance();
    private TalonSRX mTalon;
    private Controller mController = (mConst.singleControllerMode) ? Constants.driver : Constants.operator;
    private ElevatorControlMode mControlMode = ElevatorControlMode.DIRECT;
    private ElevatorControlMode mClosedLoopStategy = ElevatorControlMode.MOTION_MAGIC;

    // Tuning
    CustomPreferences mTunePrefs = new CustomPreferences("ElevatorTuning");

    boolean mTuning = true;

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

    /**
     * Initilizes the motors and the loggers.
     *
     * @return
     */
    @Override
    protected boolean init()
    {
        try
        {
            mTalon = TalonSRXFactory.createDefaultMasterTalonSRX(mConst.el_talon);
            mTalon.setNeutralMode(NeutralMode.Brake);
        } catch (TalonCommunicationErrorException e)
        {
            log("Could not initialize motor, elevator init failed! Port: " + e.getPortNumber(), Logger.LogLevel.ERROR);
            return false;
        }
//        mCanifier = new CANifier(mConst.elevatorCanifier);

        mTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, mConst.talonTimeout);
//        mTalon.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteCANifier, LimitSwitchNormal.NormallyOpen,
//                mCanifier.getDeviceID(), mConst.talonTimeout);

        if (!mTunePrefs.containsKey("magP")) mTunePrefs.putDouble("magP", 0);
        if (!mTunePrefs.containsKey("magI")) mTunePrefs.putDouble("magI", 0);
        if (!mTunePrefs.containsKey("magD")) mTunePrefs.putDouble("magD", 0);
        if (!mTunePrefs.containsKey("magF")) mTunePrefs.putDouble("magF", 0);

        if (!mTunePrefs.containsKey("posP")) mTunePrefs.putDouble("posP", 0);
        if (!mTunePrefs.containsKey("posI")) mTunePrefs.putDouble("posI", 0);
        if (!mTunePrefs.containsKey("posD")) mTunePrefs.putDouble("posD", 0);
        if (!mTunePrefs.containsKey("posF")) mTunePrefs.putDouble("posF", 0);

        if (!mTunePrefs.containsKey("set")) mTunePrefs.putDouble("set", 0);

        mTunePrefs.putInt("pos", 0);
        mTunePrefs.putInt("vel", 0);
        mTunePrefs.putInt("error", 0);

        configureController(mController);

        BadLog.createTopicStr("Elevator/Control Mode", BadLog.UNITLESS, () -> mControlMode.name(), "hide");
        BadLog.createTopic("Elevator/Output Percent", BadLog.UNITLESS, () -> mTalon.getMotorOutputPercent(), "hide");
        BadLog.createTopic("Elevator/Output Voltage Master", "V", () -> mTalon.getBusVoltage(), "hide");
        BadLog.createTopic("Elevator/Output Current Master", "I", () -> mTalon.getOutputCurrent(), "hide");
        BadLog.createTopic("Elevator/Position", "Counts", () -> (double) mTalon.getSensorCollection().getQuadraturePosition(), "hide");
        return true;
    }

    public void reset()
    {
        boolean setSucceeded = true;
        int retryCounter = 0;
        final int maxRetries = 5;
        do
        {
            setSucceeded &= mTalon.setSelectedSensorPosition(50, 0, mConst.talonTimeout) == ErrorCode.OK;
        } while (!setSucceeded || retryCounter++ < maxRetries);
    }

    @Override
    public Command getDefaultCommand()
    {
        return new Command()
        {
            ElevatorPosition desiredPosition = null, lastPos;

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
                    double rawInput = mController.get(mConst.el_rawControl);
                    lastPos = desiredPosition;
                    getDesiredPosition();
                    boolean updatePos = lastPos != desiredPosition;

                    // Temporary: for initial testing
                    mTalon.set(ControlMode.PercentOutput, rawInput);

//                if (rawInput != 0)
//                {
//                    Elevator.mInstance.stopAuxilaryCommand();
//                    mTalon.set(ControlMode.PercentOutput, rawInput);
//                    return;
//                }
//
//                if (/*desiredPosition != null && updatePos*/ mController.get(mConst.el_midCargo))
//                {
//                    setCommmandGroup(setPosition(mTunePrefs.getInt("el_set", -1), "tune"));
//                }

                    if (mTuning)
                    {
                        mTunePrefs.putInt("pos", mTalon.getSelectedSensorPosition(0));
                        mTunePrefs.putInt("vel", mTalon.getSelectedSensorVelocity(0));
                        mTunePrefs.putInt("error", mTalon.getClosedLoopError(0));
                    }

                    log("Elevator Position: " + mTalon.getSelectedSensorPosition(0), Logger.LogLevel.DEBUG);
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
            }
        };
    }

    public Command setPosition(int counts, String posName)
    {
        return new Command()
        {
            boolean isFinished = false;

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

                if (!mTuning)
                {
                    TalonSRXFactory.configurePIDF(mTalon, ElevatorControlMode.MOTION_MAGIC.getSlotID(),
                            mConst.el_motionPID.getP(),
                            mConst.el_motionPID.getI(),
                            mConst.el_motionPID.getD(),
                            mConst.el_motionPID.getF());

                    TalonSRXFactory.configurePIDF(mTalon, ElevatorControlMode.POS_PID.getSlotID(),
                            mConst.el_posPID.getP(),
                            mConst.el_posPID.getI(),
                            mConst.el_posPID.getD(),
                            mConst.el_posPID.getF());
                } else
                {
                    TalonSRXFactory.configurePIDF(mTalon, ElevatorControlMode.MOTION_MAGIC.getSlotID(),
                            mTunePrefs.getDouble("magP", 0),
                            mTunePrefs.getDouble("magI", 0),
                            mTunePrefs.getDouble("magD", 0),
                            mTunePrefs.getDouble("magF", 0));

                    TalonSRXFactory.configurePIDF(mTalon, ElevatorControlMode.MOTION_MAGIC.getSlotID(),
                            mTunePrefs.getDouble("posP", 0),
                            mTunePrefs.getDouble("posI", 0),
                            mTunePrefs.getDouble("posD", 0),
                            mTunePrefs.getDouble("posF", 0));
                }

                boolean setsSucceeded = true;
                int maxRetries = 5;
                int count = 0;

                if (mClosedLoopStategy == ElevatorControlMode.MOTION_MAGIC)
                {
                    do
                    {
                        setsSucceeded &= mTalon.configMotionCruiseVelocity(mConst.el_maxVelocity, mConst.talonTimeout) == ErrorCode.OK;
                        setsSucceeded &= mTalon.configMotionAcceleration(mConst.el_maxAcceleration, mConst.talonTimeout) == ErrorCode.OK;
                    } while (!setsSucceeded || count++ < maxRetries);

                    if (!setsSucceeded || count > maxRetries)
                    {
                        log("Could not configure motion magic constants!", Logger.LogLevel.ERROR);
                        return false;
                    }
                }
                mTalon.selectProfileSlot(mClosedLoopStategy.getSlotID(), 0);
                mTalon.set(mClosedLoopStategy.getMode(), counts);
                return true;
            }

            @Override
            public void update()
            {
                if (mTuning)
                {
                    mTunePrefs.putInt("pos", mTalon.getSelectedSensorPosition(0));
                    mTunePrefs.putInt("vel", mTalon.getSelectedSensorVelocity(0));
                    mTunePrefs.putInt("error", mTalon.getClosedLoopError(0));
                }
                isFinished = mTalon.getClosedLoopError() < mConst.el_allowedClosedLoopError;
            }

            @Override
            public void stop()
            {
                mTalon.set(ControlMode.PercentOutput, 0);
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

            @Override
            public String getName()
            {
                return "MoveForTime[T: " + time + ", P: " + power + "]";
            }

            @Override
            public boolean isFinished()
            {
                return (Timer.getFPGATimestamp() - startTime) > time;
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
                mTalon.set(ControlMode.PercentOutput, power);
            }

            @Override
            public void stop()
            {
                mTalon.set(ControlMode.PercentOutput, 0);
            }
        };
    }

    private double applyDeadband(double input, double deadband)
    {
        return (Math.abs(input) < deadband) ? 0 : input;
    }

    public void configureController(Controller controller)
    {
        /*
         * A - Move Elevator to Mid Cargo
         * B - Move Elevator to Low Cargo
         * X - Move Elevator to Mid Hatch
         * Y - Move Elevator to High Hatch
         * Start - Move Elevator to Default Position
         * Select - Move Elevator to High Cargo
         * Right Y - Manual elevator control
         */

        controller.config(mConst.el_midCargo, Button.ButtonMode.ON_PRESS);
        controller.config(mConst.el_lowCargo, Button.ButtonMode.ON_PRESS);
        controller.config(mConst.el_midHatch, Button.ButtonMode.ON_PRESS);
        controller.config(mConst.el_highHatch, Button.ButtonMode.ON_PRESS);
        controller.config(mConst.el_lowHatch, Button.ButtonMode.ON_PRESS);
        controller.config(mConst.el_highCargo, Button.ButtonMode.ON_PRESS);
        controller.config(mConst.el_rawControl, x -> -(applyDeadband(x, 0.15) * mConst.el_rawMultiplier));
    }
}
