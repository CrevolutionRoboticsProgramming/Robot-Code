package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.drivers.TalonCommunicationErrorException;
import org.frc2851.crevolib.drivers.TalonSRXFactory;
import org.frc2851.crevolib.io.Axis;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.motion.MotionProfileExecutor;
import org.frc2851.crevolib.motion.PID;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

public class DriveTrain extends Subsystem
{
    public enum DriveGear
    {
        HIGH(DoubleSolenoid.Value.kForward), LOW(DoubleSolenoid.Value.kReverse);

        private final DoubleSolenoid.Value val;
        DriveGear(DoubleSolenoid.Value val)
        {
            this.val = val;
        }
    }

    // Constants
    private final double CPR = 4096;
    private final double WHEEL_DIAMETER = 1.0d / 3.0d;
    private final double DRIVE_WIDTH = 2.5;
    private final double DEFAULT_PEAK_OUT = 1;
    private final double DEFAULT_NOMINAL_OUT = 0;
    private final int TALON_TIMEOUT = Constants.getInstance().talonTimeout;
    private final int PID_PRIMARY = 0, PID_AUX = 1;
    private final int PID_SLOT_DRIVE = 0, PID_SLOT_GYRO = 1, PID_SLOT_ENC_TURN = 2;
    private final int PIGEON_REMOTE_LEFT = 0, PIGEON_REMOTE_RIGHT = 0;

    private boolean USE_GYRO = false;

    // Motion Magic info
    // Units (Average Counts per 100ms)
    // TODO: Experimentally determine max vel and acc
    private final int MOTION_MAGIC_MAX_VEL = 15000;
    private final int MOTION_MAGIC_MAX_ACC = 3000;

    private final NeutralMode TALON_NEUTRAL_MODE = NeutralMode.Brake;

    // PID
    final boolean TUNING_PID = true;
    private PID leftMotionPID, rightMotionPID;

    // Members
    private WPI_TalonSRX mLeftMaster, mLeftSlaveA, mLeftSlaveB, mRightMaster, mRightSlaveA, mRightSlaveB;
    private PigeonIMU mPigeon;
    private DoubleSolenoid mShifterSolenoid;

    private Controller mController = Robot.driver;
    private Constants mConstants = Constants.getInstance();
    private DriveGear mCurrentGear = mConstants.defaultDriveGear;

    private static DriveTrain mInstance = new DriveTrain();

    private DriveTrain()
    {
        super("DriveTrain");
    }

    public static DriveTrain getInstance()
    {
        return mInstance;
    }

    @Override
    protected boolean init()
    {
        try {
            mLeftMaster = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.leftDriveMaster);
            mLeftSlaveA = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.leftDriveSlaveA);
            mLeftSlaveB = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.leftDriveSlaveB);
            mRightMaster = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.rightDriveMaster);
            mRightSlaveA = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.rightDriveSlaveA);
            mRightSlaveB = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.rightDriveSlaveB);
        } catch (TalonCommunicationErrorException e) {
            log("Could not initialize motor, drivetrain init failed! Port: " + e.getPortNumber(), Logger.LogLevel.ERROR);
            return false;
        }

        setNominalAndPeakOutputs(DEFAULT_NOMINAL_OUT, DEFAULT_PEAK_OUT);

        // I don't know what safety does but the messages it throws annoy me.
        mLeftMaster.setSafetyEnabled(false);
        mLeftSlaveA.setSafetyEnabled(false);
        mLeftSlaveB.setSafetyEnabled(false);
        mRightMaster.setSafetyEnabled(false);
        mRightSlaveA.setSafetyEnabled(false);
        mRightSlaveB.setSafetyEnabled(false);

        mShifterSolenoid = new DoubleSolenoid(mConstants.pcmID, mConstants.shifterForwardChannel, mConstants.shifterReverseChannel);

        // Pigeon Configuration
        if (USE_GYRO) {
            mPigeon = new PigeonIMU(0); // TODO: Add to config file
            mLeftMaster.configRemoteFeedbackFilter(mPigeon.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, PIGEON_REMOTE_LEFT, TALON_TIMEOUT);
            mRightMaster.configRemoteFeedbackFilter(mPigeon.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, PIGEON_REMOTE_RIGHT, TALON_TIMEOUT);
        }
        return true;
    }

    private void badLogInit()
    {
        BadLog.createTopic("Drivetrain/Left Percent", BadLog.UNITLESS, () -> mLeftMaster.getMotorOutputPercent(), "hide", "join:Drivetrain/Percent Outputs");
        BadLog.createTopic("Drivetrain/Right Percent", BadLog.UNITLESS, () -> mRightMaster.getMotorOutputPercent(), "hide", "join:Drivetrain/Percent Outputs");

        BadLog.createTopic("Drivetrain/Left Master Voltage", "V", () -> mLeftMaster.getBusVoltage(), "hide", "join:Drivetrain/Voltage Outputs");
        BadLog.createTopic("Drivetrain/Left Slave A Voltage", "V", () -> mLeftSlaveA.getBusVoltage(), "hide", "join:Drivetrain/Voltage Outputs");
        BadLog.createTopic("Drivetrain/Left Slave B Voltage", "V", () -> mLeftSlaveB.getBusVoltage(), "hide", "join:Drivetrain/Voltage Outputs");
        BadLog.createTopic("Drivetrain/Right Master Voltage", "V", () -> mRightMaster.getBusVoltage(), "hide", "join:Drivetrain/Voltage Outputs");
        BadLog.createTopic("Drivetrain/Right Slave A Voltage", "V", () -> mRightSlaveA.getBusVoltage(), "hide", "join:Drivetrain/Voltage Outputs");
        BadLog.createTopic("Drivetrain/Right Slave B Voltage", "V", () -> mRightSlaveB.getBusVoltage(), "hide", "join:Drivetrain/Voltage Outputs");

        BadLog.createTopic("Drivetrain/Left Master Current", "A", () -> mLeftMaster.getOutputCurrent(), "hide", "join:Drivetrain/Current Outputs");
        BadLog.createTopic("Drivetrain/Left Slave A Current", "A", () -> mLeftSlaveA.getOutputCurrent(), "hide", "join:Drivetrain/Current Outputs");
        BadLog.createTopic("Drivetrain/Left Slave B Current", "A", () -> mLeftSlaveB.getOutputCurrent(), "hide", "join:Drivetrain/Current Outputs");
        BadLog.createTopic("Drivetrain/Right Master Current", "A", () -> mRightMaster.getOutputCurrent(), "hide", "join:Drivetrain/Current Outputs");
        BadLog.createTopic("Drivetrain/Right Slave A Current", "A", () -> mRightSlaveA.getOutputCurrent(), "hide", "join:Drivetrain/Current Outputs");
        BadLog.createTopic("Drivetrain/Right Slave B Current", "A", () -> mRightSlaveB.getOutputCurrent(), "hide", "join:Drivetrain/Current Outputs");

        BadLog.createTopic("Drivetrain/Left Encoder", "counts", () -> (double) mLeftMaster.getSensorCollection().getQuadraturePosition(), "hide", "join:Drivetrain/Encoders (Pos)");
        BadLog.createTopic("Drivetrain/Right Encoder", "counts", () -> (double) mRightMaster.getSensorCollection().getQuadraturePosition(), "hide", "join:Drivetrain/Encoders (Pos)");
        BadLog.createTopic("Drivetrain/Left Velocity", "f/s", () -> ctreVelToFPS(mLeftMaster.getSensorCollection().getQuadratureVelocity()), "hide", "join:Drivetrain/Encoders (Vel)");
        BadLog.createTopic("Drivetrain/Right Velocity", "f/s", () -> ctreVelToFPS(mRightMaster.getSensorCollection().getQuadratureVelocity()), "hide", "join:Drivetrain/Encoders (Vel)");
        if (USE_GYRO) BadLog.createTopic("Drivetrain/Angle", "deg", () -> mPigeon.getFusedHeading());
    }

    private void reset()
    {
        zeroSensors();
        setLeftRightMotorOutputs(0, 0);
        mLeftMaster.setNeutralMode(TALON_NEUTRAL_MODE);
        mLeftSlaveA.setNeutralMode(TALON_NEUTRAL_MODE);
        mLeftSlaveB.setNeutralMode(TALON_NEUTRAL_MODE);
        mRightMaster.setNeutralMode(TALON_NEUTRAL_MODE);
        mRightSlaveA.setNeutralMode(TALON_NEUTRAL_MODE);
        mRightSlaveB.setNeutralMode(TALON_NEUTRAL_MODE);
    }

    private void zeroSensors()
    {
        ErrorCode code = mLeftMaster.getSensorCollection().setQuadraturePosition(0, TALON_TIMEOUT);
        mRightMaster.getSensorCollection().setQuadraturePosition(0, TALON_TIMEOUT);
        if (USE_GYRO) mPigeon.setYaw(0, TALON_TIMEOUT);
        Logger.println("Zero Sensor Code: " + code.toString(), Logger.LogLevel.DEBUG);
    }

    @Override
    public Command getDefaultCommand()
    {
        return new Command()
        {
            DifferentialDrive robotDrive = new DifferentialDrive(mLeftMaster, mRightMaster);

            final double TURN_MULT = 0.9;

            @Override
            public String getName() { return "Teleop"; }

            @Override
            public boolean isFinished() { return false; }

            @Override
            public boolean init()
            {
                reset();

                mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_PRIMARY, TALON_TIMEOUT);
                mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_PRIMARY, TALON_TIMEOUT);
                mRightMaster.setSensorPhase(true);

                robotDrive.tankDrive(0, 0);
                robotDrive.setSafetyEnabled(false);
                return true;
            }

            @Override
            public void update()
            {
                double throttle = mController.get(Axis.AxisID.LEFT_Y);
                double turn = mController.get(Axis.AxisID.RIGHT_X) * TURN_MULT;
                boolean curvatureToggle = mController.get(Button.ButtonID.RIGHT_BUMPER);
                boolean gearToggle = mController.get(Button.ButtonID.LEFT_BUMPER);

                DriveGear requestedGear = (gearToggle) ? DriveGear.HIGH : DriveGear.LOW;
                if (requestedGear != mCurrentGear) setCommmandGroup(setDriveGear(requestedGear));

                robotDrive.curvatureDrive(throttle, turn, curvatureToggle);

                // Encoder Test
                String encTest = "Left(" + mLeftMaster.getSelectedSensorPosition(0) + "," + mLeftMaster.getSelectedSensorVelocity(0) +
                        "), Right(" + mRightMaster.getSelectedSensorPosition(0) + "," + mRightMaster.getSelectedSensorVelocity(0) + ")";
                System.out.println(encTest);
            }

            @Override
            public void stop() {
                reset();
            }
        };
    }

    public Command driveDistanceMotionProfile(double distance)
    {
        return new Command()
        {
            double targetAngle;
            double targetPos;

            final double UNITS_PER_ROTATION = 10000.0d;
            // Scales the encoder difference reading to 3600 counts per rotation
            final double turnCoeff = 3600.0d / UNITS_PER_ROTATION;

            @Override
            public String getName() {
                return "DriveDistanceMotionProfile[" + distance + "]";
            }

            @Override
            public boolean isFinished() {
                return Math.abs(targetPos - mRightMaster.getSelectedSensorPosition(0)) < 1024;
            }

            @Override
            public boolean init()
            {
                double startTime = Timer.getFPGATimestamp();
                // Sensor Config
                mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_PRIMARY, TALON_TIMEOUT);

                mLeftMaster.configRemoteFeedbackFilter(mRightMaster.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor,
                        0, TALON_TIMEOUT);
                mLeftMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, TALON_TIMEOUT);
                mLeftMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, TALON_TIMEOUT);
                mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, PID_PRIMARY, TALON_TIMEOUT);
                mLeftMaster.configSelectedFeedbackCoefficient(0.5, PID_PRIMARY, TALON_TIMEOUT);

                mLeftMaster.selectProfileSlot(PID_SLOT_DRIVE, PID_PRIMARY);

                if (USE_GYRO)
                {
                    mLeftMaster.configRemoteFeedbackFilter(mPigeon.getDeviceID(), RemoteSensorSource.Pigeon_Yaw,
                            1, TALON_TIMEOUT);
                    mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, 1, TALON_TIMEOUT);
                    mLeftMaster.configSelectedFeedbackCoefficient(3600.0d / 8192.0d, 1, TALON_TIMEOUT);

                    mLeftMaster.selectProfileSlot(PID_SLOT_GYRO, PID_AUX);
                } else {
                    mLeftMaster.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, TALON_TIMEOUT);
                    mLeftMaster.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor1, TALON_TIMEOUT);
                    mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, PID_AUX, TALON_TIMEOUT);
                    mLeftMaster.configSelectedFeedbackCoefficient(turnCoeff, 1, TALON_TIMEOUT);

                    mLeftMaster.selectProfileSlot(PID_SLOT_ENC_TURN, PID_AUX);
                }

                mLeftMaster.configAuxPIDPolarity(false, TALON_TIMEOUT);

                // Motion Magic Config
                mLeftMaster.configMotionAcceleration(MOTION_MAGIC_MAX_ACC, TALON_TIMEOUT);
                mLeftMaster.configMotionCruiseVelocity(MOTION_MAGIC_MAX_VEL, TALON_TIMEOUT);

                targetAngle = mLeftMaster.getSelectedSensorPosition(1);
                targetPos = distanceToCounts(distance) + mLeftMaster.getSelectedSensorPosition(0);
                Logger.println("Time for MMG Init: " + (Timer.getFPGATimestamp() - startTime), Logger.LogLevel.DEBUG);
                return true;
            }

            @Override
            public void update() {
                mLeftMaster.set(ControlMode.MotionMagic, targetPos, DemandType.AuxPID, targetAngle);
                mRightMaster.follow(mLeftMaster, FollowerType.AuxOutput1);
            }

            @Override
            public void stop() { reset(); }
        };
    }

    public Command turnToAngleEncoder(double angle)
    {
        return new Command()
        {
            int counts = (int)(((Math.toRadians(angle) * DRIVE_WIDTH) * 0.5) / (WHEEL_DIAMETER * Math.PI) * CPR);

            @Override
            public String getName() { return "TurnAngle[" + angle + "]"; }

            @Override
            public boolean isFinished() {
                return mLeftMaster.getClosedLoopError(0) < 512;
            }

            @Override
            public boolean init() {
                int targetPos = mLeftMaster.getSelectedSensorPosition(0) + counts;
                setPID(mLeftMaster, 0, new PID(0, 0, 0, 0));
                setPID(mRightMaster, 0, new PID(0, 0, 0, 0));
                mLeftMaster.set(ControlMode.Position, targetPos);
                mLeftSlaveA.set(ControlMode.Follower, mLeftMaster.getDeviceID());
                return true;
            }

            @Override
            public void update() {
                mRightMaster.set(ControlMode.PercentOutput, -mLeftMaster.getMotorOutputPercent());
            }

            @Override
            public void stop() {
                reset();
            }
        };
    }

    public Command driveTime(double time, double power)
    {
        return driveTime(time, power, power);
    }

    public Command driveTime(double time, double leftPower, double rightPower)
    {
        return new Command() {
            Timer timer = new Timer();

            @Override
            public String getName() { return "DriveTime[" + time + "s, " + leftPower + ", " + rightPower + "]"; }

            @Override
            public boolean isFinished() { return timer.get() > time; }

            @Override
            public boolean init()
            {
                timer.start();
                return true;
            }

            @Override
            public void update() { setLeftRightMotorOutputs(leftPower, rightPower); }

            @Override
            public void stop()
            {
                timer.stop();
                reset();
            }
        };
    }

    public Command runMotionProfile(String leftProfile, String rightProfile)
    {
        return new Command()
        {
            MotionProfileExecutor left, right;
            boolean profileNull = false;

            @Override
            public String getName() { return "MotionProfile[" + leftProfile + ", " + rightProfile + "]"; }

            @Override
            public boolean isFinished()
            {
                // TODO: Double check that the flag is triggering
                SetValueMotionProfile leftSet, rightSet;
                leftSet = left.getSetValue();
                rightSet = right.getSetValue();

                return (leftSet == SetValueMotionProfile.Hold && rightSet == SetValueMotionProfile.Hold);
            }

            @Override
            public boolean init()
            {
                try {
                    left = new MotionProfileExecutor(Robot.getMotionProfile(leftProfile), mLeftMaster, false);
                    right = new MotionProfileExecutor(Robot.getMotionProfile(rightProfile), mRightMaster, false);
                } catch (NullPointerException e) {
                    return false;
                }

                setPID(mLeftMaster, 0, leftMotionPID);
                setPID(mRightMaster, 0, rightMotionPID);

                //TODO: Feedback Coeff
                mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_PRIMARY, TALON_TIMEOUT);
                mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_PRIMARY, TALON_TIMEOUT);
                if (USE_GYRO) {
                    mLeftMaster.configRemoteFeedbackFilter(mPigeon.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, 0, TALON_TIMEOUT);
                    mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, PID_AUX, TALON_TIMEOUT);

                    mRightMaster.configRemoteFeedbackFilter(mPigeon.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, 0, TALON_TIMEOUT);
                    mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, PID_AUX, TALON_TIMEOUT);
                }

                left.start();
                right.start();
                return true;
            }

            @Override
            public void update()
            {
                left.update();
                right.update();

                mLeftMaster.set(ControlMode.MotionProfile, left.getSetValue().value);
                mRightMaster.set(ControlMode.MotionProfile, right.getSetValue().value);
            }

            @Override
            public void stop()
            {
                left.reset();
                right.reset();
                reset();
            }
        };
    }

    public Command setDriveGear(DriveGear gear) {
        return new Command() {
            @Override
            public String getName() { return "SetDriveGear[" + gear.name() + "]"; }

            @Override
            public boolean isFinished() { return true; }

            @Override
            public boolean init()
            {
                mShifterSolenoid.set(gear.val);
                return true;
            }

            @Override
            public void update() { }

            @Override
            public void stop() { }
        };
    }

    private void setPID(TalonSRX talon, int slot, PID pid)
    {
        talon.config_kP(slot, pid.p, TALON_TIMEOUT);
        talon.config_kI(slot, pid.i, TALON_TIMEOUT);
        talon.config_kD(slot, pid.d, TALON_TIMEOUT);
        talon.config_kF(slot, pid.f, TALON_TIMEOUT);
    }

    private void setNominalAndPeakOutputs(double nominal, double peak)
    {
        mLeftMaster.configNominalOutputForward(nominal, TALON_TIMEOUT);
        mLeftMaster.configPeakOutputForward(peak, TALON_TIMEOUT);
        mLeftMaster.configNominalOutputReverse(-nominal, TALON_TIMEOUT);
        mLeftMaster.configPeakOutputReverse(-peak, TALON_TIMEOUT);

        mRightMaster.configNominalOutputForward(nominal, TALON_TIMEOUT);
        mRightMaster.configPeakOutputForward(peak, TALON_TIMEOUT);
        mRightMaster.configNominalOutputReverse(-nominal, TALON_TIMEOUT);
        mRightMaster.configPeakOutputReverse(-peak, TALON_TIMEOUT);
    }

    private void setLeftRightMotorOutputs(double left, double right)
    {
        mLeftMaster.set(ControlMode.PercentOutput, left);
        mRightMaster.set(ControlMode.PercentOutput, right);
    }

    private void configureController(Controller controller)
    {
        controller.config(Axis.AxisID.LEFT_Y, x -> applyDeadband(x, 0.15)); // Throttle
        controller.config(Axis.AxisID.RIGHT_X, x -> applyDeadband(x, 0.15)); // Turn
        controller.config(Button.ButtonID.RIGHT_BUMPER, Button.ButtonMode.TOGGLE); // Curvature
        controller.config(Button.ButtonID.LEFT_BUMPER, Button.ButtonMode.TOGGLE); // Shifter
    }

    private int distanceToCounts(double distance)
    {
        return (int)(distance / (CPR * WHEEL_DIAMETER * Math.PI));
    }

    private double applyDeadband(double val, double deadband) {
        return (Math.abs(val) < deadband) ? 0 : val;
    }

    // counts / 100ms
    private double ctreVelToFPS(int ctreVel) {
        return ((ctreVel * 10) / CPR) * WHEEL_DIAMETER * Math.PI;
    }
}