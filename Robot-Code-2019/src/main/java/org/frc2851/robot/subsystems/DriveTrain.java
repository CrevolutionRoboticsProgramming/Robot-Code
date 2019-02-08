package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import org.frc2851.crevolib.ElementNotFoundException;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.configuration.RobotConfig;
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
import org.jdom2.DataConversionException;

public class DriveTrain extends Subsystem
{
    public enum DriveGear { HIGH, LOW }

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
    PID leftMotionPID, rightMotionPID;

    // Members
    private WPI_TalonSRX _leftMaster, _leftSlaveA, _leftSlaveB, _rightMaster, _rightSlaveA, _rightSlaveB;
    private Controller _controller = Robot.driver;
    private PigeonIMU _pigeon;
    private Constants mConstants = Constants.getInstance();
    private DriveGear mDriveGear = mConstants.defaultDriveGear;

    private static DriveTrain _instance = new DriveTrain();
    private DriveTrain()
    {
        super("DriveTrain");
    }
    public static DriveTrain getInstance() { return _instance; }

    @Override
    protected boolean init()
    {
//        try {
//            _leftMaster = _config.getWpiTalon("talonLeftA");
//            _leftSlaveA = _config.getWpiTalon("talonLeftB");
//            _rightMaster = _config.getWpiTalon("talonRightA");
//             = _config.getWpiTalon("talonRightB");
//        } catch (ElementNotFoundException e) {
//            return false;
//        }

        _leftMaster = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.leftDriveMaster);
        _leftSlaveA = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.leftDriveSlaveA);
        _leftSlaveB = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.leftDriveSlaveB);
        _rightMaster = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.rightDriveMaster);
        _rightSlaveA = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.rightDriveSlaveA);
        _rightSlaveB = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.rightDriveSlaveB);

//        try {
//            if (TUNING_PID) {
//                Preferences p = Preferences.getInstance();
//                leftMotionPID = new PID(p.getDouble("leftP", 0), p.getDouble("leftI", 0),
//                        p.getDouble("leftD", 0), p.getDouble("leftF", 0));
//                rightMotionPID = new PID(p.getDouble("rightP", 0), p.getDouble("rightI", 0),
//                        p.getDouble("rightD", 0), p.getDouble("rightF", 0));
//            } else {
//                leftMotionPID = _config.getPID("leftMotionPID");
//                rightMotionPID = _config.getPID("rightMotionPID");
//            }
//        } catch (ElementNotFoundException | DataConversionException e) {
//            Logger.println("Could not read PID values DriveTrain", Logger.LogLevel.ERROR);
//        }

        setNominalAndPeakOutputs(DEFAULT_NOMINAL_OUT, DEFAULT_PEAK_OUT);

        // Controller configuration
        _controller.config(Axis.AxisID.LEFT_Y); // Throttle
        _controller.config(Axis.AxisID.RIGHT_X); // Turn
        _controller.config(Button.ButtonID.RIGHT_BUMPER, Button.ButtonMode.TOGGLE); // Curvature Toggle

        // I don't know what safety does but the messages it throws annoy me.
        _leftMaster.setSafetyEnabled(false);
        _leftSlaveA.setSafetyEnabled(false);
        _leftSlaveB.setSafetyEnabled(false);
        _rightMaster.setSafetyEnabled(false);
        _rightSlaveA.setSafetyEnabled(false);
        _rightSlaveB.setSafetyEnabled(false);

        // Pigeon Configuration
        if (USE_GYRO) {
            _pigeon = new PigeonIMU(0); // TODO: Add to config file
            _leftMaster.configRemoteFeedbackFilter(_pigeon.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, PIGEON_REMOTE_LEFT, TALON_TIMEOUT);
            _rightMaster.configRemoteFeedbackFilter(_pigeon.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, PIGEON_REMOTE_RIGHT, TALON_TIMEOUT);
        }

        reset(); // Probably unnecessary. Worth the lost cycles for certainty.

        BadLog.createTopic("Drivetrain/Left Percent", BadLog.UNITLESS, () -> _leftMaster.getMotorOutputPercent(), "hide", "join:Drivetrain/Percent Outputs");
        BadLog.createTopic("Drivetrain/Right Percent", BadLog.UNITLESS, () -> _rightMaster.getMotorOutputPercent(), "hide", "join:Drivetrain/Percent Outputs");

        BadLog.createTopic("Drivetrain/Left Master Voltage", "V", () -> _leftMaster.getBusVoltage(), "hide", "join:Drivetrain/Voltage Outputs");
        BadLog.createTopic("Drivetrain/Left Slave A Voltage", "V", () -> _leftSlaveA.getBusVoltage(), "hide", "join:Drivetrain/Voltage Outputs");
        BadLog.createTopic("Drivetrain/Left Slave B Voltage", "V", () -> _leftSlaveB.getBusVoltage(), "hide", "join:Drivetrain/Voltage Outputs");
        BadLog.createTopic("Drivetrain/Right Master Voltage", "V", () -> _rightMaster.getBusVoltage(), "hide", "join:Drivetrain/Voltage Outputs");
        BadLog.createTopic("Drivetrain/Right Slave A Voltage", "V", () -> _rightSlaveA.getBusVoltage(), "hide", "join:Drivetrain/Voltage Outputs");
        BadLog.createTopic("Drivetrain/Right Slave B Voltage", "V", () -> _rightSlaveB.getBusVoltage(), "hide", "join:Drivetrain/Voltage Outputs");

        BadLog.createTopic("Drivetrain/Left Master Current", "A", () -> _leftMaster.getOutputCurrent(), "hide", "join:Drivetrain/Current Outputs");
        BadLog.createTopic("Drivetrain/Left Slave A Current", "A", () -> _leftSlaveA.getOutputCurrent(), "hide", "join:Drivetrain/Current Outputs");
        BadLog.createTopic("Drivetrain/Left Slave B Current", "A", () -> _leftSlaveB.getOutputCurrent(), "hide", "join:Drivetrain/Current Outputs");
        BadLog.createTopic("Drivetrain/Right Master Current", "A", () -> _rightMaster.getOutputCurrent(), "hide", "join:Drivetrain/Current Outputs");
        BadLog.createTopic("Drivetrain/Right Slave A Current", "A", () -> _rightSlaveA.getOutputCurrent(), "hide", "join:Drivetrain/Current Outputs");
        BadLog.createTopic("Drivetrain/Right Slave B Current", "A", () -> _rightSlaveB.getOutputCurrent(), "hide", "join:Drivetrain/Current Outputs");

        BadLog.createTopic("Drivetrain/Left Encoder", "counts", () -> (double) _leftMaster.getSensorCollection().getQuadraturePosition(), "hide", "join:Drivetrain/Encoders (Pos)");
        BadLog.createTopic("Drivetrain/Right Encoder", "counts", () -> (double) _rightMaster.getSensorCollection().getQuadraturePosition(), "hide", "join:Drivetrain/Encoders (Pos)");
        BadLog.createTopic("Drivetrain/Left Velocity", "f/s", () -> ctreVelToFPS(_leftMaster.getSensorCollection().getQuadratureVelocity()), "hide", "join:Drivetrain/Encoders (Vel)");
        BadLog.createTopic("Drivetrain/Right Velocity", "f/s", () -> ctreVelToFPS(_rightMaster.getSensorCollection().getQuadratureVelocity()), "hide", "join:Drivetrain/Encoders (Vel)");
        if (USE_GYRO) BadLog.createTopic("Drivetrain/Angle", "deg", () -> _pigeon.getFusedHeading());
        return true;
    }

    private void reset()
    {
        zeroSensors();
        setLeftRightMotorOutputs(0, 0);
        _leftMaster.setNeutralMode(TALON_NEUTRAL_MODE);
        _leftSlaveA.setNeutralMode(TALON_NEUTRAL_MODE);
        _leftSlaveB.setNeutralMode(TALON_NEUTRAL_MODE);
        _rightMaster.setNeutralMode(TALON_NEUTRAL_MODE);
        _rightSlaveA.setNeutralMode(TALON_NEUTRAL_MODE);
        _rightSlaveB.setNeutralMode(TALON_NEUTRAL_MODE);
    }

    private void zeroSensors()
    {
        ErrorCode code = _leftMaster.getSensorCollection().setQuadraturePosition(0, TALON_TIMEOUT);
        _rightMaster.getSensorCollection().setQuadraturePosition(0, TALON_TIMEOUT);
        if (USE_GYRO) _pigeon.setYaw(0, TALON_TIMEOUT);
        Logger.println("Zero Sensor Code: " + code.toString(), Logger.LogLevel.DEBUG);
    }

    @Override
    public Command getDefaultCommand()
    {
        return new Command()
        {
            DifferentialDrive robotDrive = new DifferentialDrive(_leftMaster, _rightMaster);

            final double TURN_MULT = 0.9;
            final double DEADBAND_VERT = 0.11;
            final double DEADBAND_HORZ = 0.11;

            @Override
            public String getName() { return "Teleop"; }

            @Override
            public boolean isFinished() { return false; }

            @Override
            public boolean init()
            {
                reset();

                _leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_PRIMARY, TALON_TIMEOUT);
                _rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_PRIMARY, TALON_TIMEOUT);
                _rightMaster.setSensorPhase(true);

                robotDrive.tankDrive(0, 0);
                robotDrive.setSafetyEnabled(false);
                return true;
            }

            @Override
            public void update()
            {
                double throttle = applyDeadband(_controller.get(Axis.AxisID.LEFT_Y), DEADBAND_VERT);
                double turn = applyDeadband(_controller.get(Axis.AxisID.RIGHT_X) * TURN_MULT, DEADBAND_HORZ);
                boolean curvatureToggle = _controller.get(Button.ButtonID.RIGHT_BUMPER);

                robotDrive.curvatureDrive(throttle, turn, curvatureToggle);

                // Encoder Test
                String encTest = "Left(" + _leftMaster.getSelectedSensorPosition(0) + "," + _leftMaster.getSelectedSensorVelocity(0) +
                        "), Right(" + _rightMaster.getSelectedSensorPosition(0) + "," + _rightMaster.getSelectedSensorVelocity(0) + ")";
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
                return Math.abs(targetPos - _rightMaster.getSelectedSensorPosition(0)) < 1024;
            }

            @Override
            public boolean init()
            {
                double startTime = Timer.getFPGATimestamp();
                // Sensor Config
                _rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_PRIMARY, TALON_TIMEOUT);

                _leftMaster.configRemoteFeedbackFilter(_rightMaster.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor,
                        0, TALON_TIMEOUT);
                _leftMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, TALON_TIMEOUT);
                _leftMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, TALON_TIMEOUT);
                _leftMaster.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, PID_PRIMARY, TALON_TIMEOUT);
                _leftMaster.configSelectedFeedbackCoefficient(0.5, PID_PRIMARY, TALON_TIMEOUT);

                _leftMaster.selectProfileSlot(PID_SLOT_DRIVE, PID_PRIMARY);

                if (USE_GYRO)
                {
                    _leftMaster.configRemoteFeedbackFilter(_pigeon.getDeviceID(), RemoteSensorSource.Pigeon_Yaw,
                            1, TALON_TIMEOUT);
                    _leftMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, 1, TALON_TIMEOUT);
                    _leftMaster.configSelectedFeedbackCoefficient(3600.0d / 8192.0d, 1, TALON_TIMEOUT);

                    _leftMaster.selectProfileSlot(PID_SLOT_GYRO, PID_AUX);
                } else {
                    _leftMaster.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, TALON_TIMEOUT);
                    _leftMaster.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor1, TALON_TIMEOUT);
                    _leftMaster.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, PID_AUX, TALON_TIMEOUT);
                    _leftMaster.configSelectedFeedbackCoefficient(turnCoeff, 1, TALON_TIMEOUT);

                    _leftMaster.selectProfileSlot(PID_SLOT_ENC_TURN, PID_AUX);
                }

                _leftMaster.configAuxPIDPolarity(false, TALON_TIMEOUT);

                // Motion Magic Config
                _leftMaster.configMotionAcceleration(MOTION_MAGIC_MAX_ACC, TALON_TIMEOUT);
                _leftMaster.configMotionCruiseVelocity(MOTION_MAGIC_MAX_VEL, TALON_TIMEOUT);

                targetAngle = _leftMaster.getSelectedSensorPosition(1);
                targetPos = distanceToCounts(distance) + _leftMaster.getSelectedSensorPosition(0);
                Logger.println("Time for MMG Init: " + (Timer.getFPGATimestamp() - startTime), Logger.LogLevel.DEBUG);
                return true;
            }

            @Override
            public void update() {
                _leftMaster.set(ControlMode.MotionMagic, targetPos, DemandType.AuxPID, targetAngle);
                _rightMaster.follow(_leftMaster, FollowerType.AuxOutput1);
            }

            @Override
            public void stop() { reset(); }
        };
    }

    public Command turnToAngleGyro(double angle, double power)
    {
        return new Command()
        {
            @Override
            public String getName() { return "TurnAngle[" + angle + "]"; }

            @Override
            public boolean isFinished() {
                return false;
            }

            @Override
            public boolean init() {
                return true;
            }

            @Override
            public void update() {

            }

            @Override
            public void stop() {

            }
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
                return _leftMaster.getClosedLoopError(0) < 512;
            }

            @Override
            public boolean init() {
                int targetPos = _leftMaster.getSelectedSensorPosition(0) + counts;
                setPID(_leftMaster, 0, new PID(0, 0, 0, 0));
                setPID(_rightMaster, 0, new PID(0, 0, 0, 0));
                _leftMaster.set(ControlMode.Position, targetPos);
                _leftSlaveA.set(ControlMode.Follower, _leftMaster.getDeviceID());
                return true;
            }

            @Override
            public void update() {
                _rightMaster.set(ControlMode.PercentOutput, -_leftMaster.getMotorOutputPercent());
            }

            @Override
            public void stop() {
                reset();
            }
        };
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
                setLeftRightMotorOutputs(leftPower, rightPower);
                return true;
            }

            @Override
            public void update() { }

            @Override
            public void stop()
            {
                timer.stop();
                reset();
            }
        };
    }

    public Command driveDistance(double distance, double maxPower)
    {
        return new Command()
        {
            boolean isFinished = false;
            int counts = distanceToCounts(distance);
            @Override
            public String getName() { return "DriveDistance[" + distance + " feet, " + maxPower + "]"; }

            @Override
            public boolean isFinished() {
                return isFinished;
            }

            @Override
            public boolean init()
            {
                reset();
                setPID(_leftMaster, 0, new PID(0, 0, 0, 0));
                setPID(_rightMaster, 0, new PID(0, 0, 0, 0));
                setNominalAndPeakOutputs(DEFAULT_NOMINAL_OUT, maxPower);

                _leftMaster.set(ControlMode.Position, counts);
                _rightMaster.set(ControlMode.Position, counts);
                return true;
            }

            @Override
            public void update()
            {
                // Will finish when the motor is within 1/4 of a rotation
                if (Math.abs(counts - _leftMaster.getSelectedSensorPosition(0)) < 1024) isFinished = true;
            }

            @Override
            public void stop() {
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
                    left = new MotionProfileExecutor(Robot.getMotionProfile(leftProfile), _leftMaster, false);
                    right = new MotionProfileExecutor(Robot.getMotionProfile(rightProfile), _rightMaster, false);
                } catch (NullPointerException e) {
                    return false;
                }

                setPID(_leftMaster, 0, leftMotionPID);
                setPID(_rightMaster, 0, rightMotionPID);

                //TODO: Feedback Coeff
                _leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_PRIMARY, TALON_TIMEOUT);
                _rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_PRIMARY, TALON_TIMEOUT);
                if (USE_GYRO) {
                    _leftMaster.configRemoteFeedbackFilter(_pigeon.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, 0, TALON_TIMEOUT);
                    _leftMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, PID_AUX, TALON_TIMEOUT);

                    _rightMaster.configRemoteFeedbackFilter(_pigeon.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, 0, TALON_TIMEOUT);
                    _rightMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, PID_AUX, TALON_TIMEOUT);
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

                _leftMaster.set(ControlMode.MotionProfile, left.getSetValue().value);
                _rightMaster.set(ControlMode.MotionProfile, right.getSetValue().value);
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

    // TODO: Implement Motion Profile Arc
    public Command runMotionProfileArc(String left, String right)
    {
        return new Command() {
            @Override
            public String getName() {
                return "RunMotionProfileArc(" + left + ", " + right + ")";
            }

            @Override
            public boolean isFinished() {
                return false;
            }

            @Override
            public boolean init()
            {
                return true;
            }

            @Override
            public void update() {

            }

            @Override
            public void stop() {

            }
        };
    }

    private void setDriveGear(DriveGear gear) {
        mDriveGear = gear;
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
        _leftMaster.configNominalOutputForward(nominal, TALON_TIMEOUT);
        _leftMaster.configPeakOutputForward(peak, TALON_TIMEOUT);
        _leftMaster.configNominalOutputReverse(-nominal, TALON_TIMEOUT);
        _leftMaster.configPeakOutputReverse(-peak, TALON_TIMEOUT);

        _rightMaster.configNominalOutputForward(nominal, TALON_TIMEOUT);
        _rightMaster.configPeakOutputForward(peak, TALON_TIMEOUT);
        _rightMaster.configNominalOutputReverse(-nominal, TALON_TIMEOUT);
        _rightMaster.configPeakOutputReverse(-peak, TALON_TIMEOUT);
    }

    private void setLeftRightMotorOutputs(double left, double right)
    {
        _leftMaster.set(ControlMode.PercentOutput, left);
        _rightMaster.set(ControlMode.PercentOutput, right);
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
