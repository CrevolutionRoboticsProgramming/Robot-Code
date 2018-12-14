package org.frc2851.vulcan.subsystems;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
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
import org.frc2851.crevolib.io.Axis;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.motion.MotionProfileExecutor;
import org.frc2851.crevolib.motion.PID;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.vulcan.Robot;
import org.jdom2.DataConversionException;

public class DriveTrain extends Subsystem
{
    // Constants
    private final double CPR = 4096;
    private final double WHEEL_DIAMETER = 1.0d / 3.0d;
    private final double DRIVE_WIDTH = 2.5;
    private final double DEFAULT_PEAK_OUT = 1;
    private final double DEFAULT_NOMINAL_OUT = 0;
    private final int TALON_TIMEOUT = 0;

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
    private WPI_TalonSRX _talonLeftA, _talonLeftB, _talonRightA, _talonRightB;
    private Controller _controller = Robot.driver;
    private PigeonIMU _pigeon;
    private RobotConfig _config = new RobotConfig();

    public DriveTrain()
    {
        super("DriveTrain");
        _config.readFile("/home/lvuser/configuration/robot.xml");
    }

    @Override
    protected boolean init()
    {

        try {
            _talonLeftA = _config.getWpiTalon("talonLeftA");
            _talonLeftB = _config.getWpiTalon("talonLeftB");
            _talonRightA = _config.getWpiTalon("talonRightA");
            _talonRightB = _config.getWpiTalon("talonRightB");
        } catch (ElementNotFoundException e) {
            return false;
        }

        try {
            if (TUNING_PID) {
                Preferences p = Preferences.getInstance();
                leftMotionPID = new PID(p.getDouble("leftP", 0), p.getDouble("leftI", 0),
                        p.getDouble("leftD", 0), p.getDouble("leftF", 0));
                rightMotionPID = new PID(p.getDouble("rightP", 0), p.getDouble("rightI", 0),
                        p.getDouble("rightD", 0), p.getDouble("rightF", 0));
            } else {
                leftMotionPID = _config.getPID("leftMotionPID");
                rightMotionPID = _config.getPID("rightMotionPID");
            }
        } catch (ElementNotFoundException | DataConversionException e) {
            Logger.println("Could not read PID values DriveTrain", Logger.LogLevel.ERROR);
        }

        // Motor Controller Configuration
        _talonLeftA.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TALON_TIMEOUT);
        _talonLeftA.setSelectedSensorPosition(0, 0, TALON_TIMEOUT);

        _talonRightA.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TALON_TIMEOUT);
        _talonRightA.setSelectedSensorPosition(0, 0, TALON_TIMEOUT);
        _talonRightA.setSensorPhase(true);

        setNominalAndPeakOutputs(DEFAULT_NOMINAL_OUT, DEFAULT_PEAK_OUT);

        _talonLeftB.set(ControlMode.Follower, _talonLeftA.getDeviceID());
        _talonRightB.set(ControlMode.Follower, _talonRightA.getDeviceID());

        // Controller configuration
        _controller.config(Axis.AxisID.LEFT_Y); // Throttle
        _controller.config(Axis.AxisID.RIGHT_X); // Turn
        _controller.config(Button.ButtonID.RIGHT_BUMPER, Button.ButtonMode.TOGGLE); // Curvature Toggle

        // Pigeon Config
        _pigeon = new PigeonIMU(0);
        _talonLeftA.configRemoteFeedbackFilter(0, RemoteSensorSource.Pigeon_Yaw, 1, 10);

        reset(); // Probably unnecessary. Worth the lost cycles for certainty.
        return true;
    }

    private void reset()
    {
        zeroSensors();
        setLeftRightMotorOutputs(0, 0);
        _talonLeftA.setNeutralMode(TALON_NEUTRAL_MODE);
        _talonLeftB.setNeutralMode(TALON_NEUTRAL_MODE);
        _talonRightA.setNeutralMode(TALON_NEUTRAL_MODE);
        _talonRightB.setNeutralMode(TALON_NEUTRAL_MODE);
    }

    private void zeroSensors()
    {
        _talonLeftA.setSelectedSensorPosition(0, 0, 0);
        _talonRightA.setSelectedSensorPosition(0, 0, 0);
    }

    @Override
    public Command getTeleopCommand()
    {
        return new Command()
        {
            DifferentialDrive robotDrive = new DifferentialDrive(
                    new SpeedControllerGroup(_talonLeftA, _talonLeftB),
                    new SpeedControllerGroup(_talonRightA, _talonRightB)
            );

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
                robotDrive.tankDrive(0, 0);
                robotDrive.setSafetyEnabled(false);
                return true;
            }

            @Override
            public void update()
            {
                double throttle = applyDeadband(_controller.get(Axis.AxisID.LEFT_Y, null), DEADBAND_VERT);
                double turn = applyDeadband(_controller.get(Axis.AxisID.RIGHT_X, null) * TURN_MULT, DEADBAND_HORZ);
                boolean curvatureToggle = _controller.get(Button.ButtonID.RIGHT_BUMPER);

                robotDrive.curvatureDrive(throttle, turn, curvatureToggle);

                // Encoder Test
                System.out.println("[DT]: EncL(" + _talonLeftA.getSelectedSensorPosition(0) + ", " +
                        _talonLeftA.getSelectedSensorVelocity(0) + "), EncR(" + _talonRightA.getSelectedSensorPosition(0) +
                        ", " + _talonRightA.getSelectedSensorVelocity(0) + ")");
            }

            @Override
            public void stop() {
                reset();
            }
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
                return _talonLeftA.getClosedLoopError(0) < 512;
            }

            @Override
            public boolean init() {
                int targetPos = _talonLeftA.getSelectedSensorPosition(0) + counts;
                setPID(_talonLeftA, 0, new PID(0, 0, 0, 0));
                setPID(_talonRightA, 0, new PID(0, 0, 0, 0));
                _talonLeftA.set(ControlMode.Position, targetPos);
                _talonLeftB.set(ControlMode.Follower, _talonLeftA.getDeviceID());
                return true;
            }

            @Override
            public void update() {
                _talonRightA.set(ControlMode.PercentOutput, -_talonLeftA.getMotorOutputPercent());
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
                setPID(_talonLeftA, 0, new PID(0, 0, 0, 0));
                setPID(_talonRightA, 0, new PID(0, 0, 0, 0));
                setNominalAndPeakOutputs(DEFAULT_NOMINAL_OUT, maxPower);

                _talonLeftA.set(ControlMode.Position, counts);
                _talonRightA.set(ControlMode.Position, counts);
                _talonLeftB.set(ControlMode.Follower, _talonLeftA.getDeviceID());
                _talonRightB.set(ControlMode.Follower, _talonRightA.getDeviceID());
                return true;
            }

            @Override
            public void update()
            {
                // Will finish when the motor is within 1/4 of a rotation
                if (Math.abs(counts - _talonLeftA.getSelectedSensorPosition(0)) < 1024) isFinished = true;
            }

            @Override
            public void stop() {
                reset();
            }
        };
    }

//    public Command driveDistanceMotionMagic(double distance, double maxPower)
//    {
//        return new Command()
//        {
//            boolean isFinished = false;
//            int counts = distanceToCounts(distance);
//            final int MAX_ACC = _config.getInt(""
//            @Override
//            public String getName() { return "DriveDistanceMM[\" + distance + \" feet, \" + power + \"]"; }
//
//            @Override
//            public boolean isFinished()
//            {
//                return isFinished;
//            }
//
//            @Override
//            public void init()
//            {
//                reset();
//                setPID(_talonLeftA, 0, 0.2, 0, 0, 0);
//                setPID(_talonRightA, 0, 0.2, 0, 0, 0);
//                setNominalAndPeakOutputs(DEFAULT_NOMINAL_OUT, maxPower);
//
//                _talonLeftA.configMotionCruiseVelocity(MOTION_MAGIC_MAX_VEL, TALON_TIMEOUT);
//                _talonRightA.configMotionCruiseVelocity(MOTION_MAGIC_MAX_VEL, TALON_TIMEOUT);
//                _talonLeftA.configMotionAcceleration(MOTION_MAGIC_MAX_ACC, TALON_TIMEOUT);
//                _talonRightA.configMotionAcceleration(MOTION_MAGIC_MAX_ACC, TALON_TIMEOUT);
//
//                _talonLeftA.set(ControlMode.MotionMagic, counts);
//                _talonLeftB.set(ControlMode.Follower, _talonLeftA.getDeviceID());
//                _talonRightA.set(ControlMode.MotionMagic, counts);
//                _talonRightB.set(ControlMode.Follower, _talonRightA.getDeviceID());
//            }
//
//            @Override
//            public void update()
//            {
//                // Will finish when the motor is within 1/4 of a rotation
//                if (Math.abs(counts - _talonLeftA.getSelectedSensorPosition(0)) < 1024) isFinished = true;
//            }
//
//            @Override
//            public void stop()
//            {
//                reset();
//            }
//        };
//    }

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
                    left = new MotionProfileExecutor(Robot.getMotionProfile(leftProfile), _talonLeftA, false);
                    right = new MotionProfileExecutor(Robot.getMotionProfile(rightProfile), _talonRightA, false);
                } catch (NullPointerException e) {
                    return false;
                }

                setPID(_talonLeftA, 0, leftMotionPID);
                setPID(_talonRightA, 0, rightMotionPID);

                left.start();
                right.start();
                return true;
            }

            @Override
            public void update()
            {
                left.update();
                right.update();

                _talonLeftA.set(ControlMode.MotionProfile, left.getSetValue().value);
                _talonRightA.set(ControlMode.MotionProfile, right.getSetValue().value);
                _talonLeftB.set(ControlMode.Follower, _talonLeftA.getDeviceID());
                _talonRightB.set(ControlMode.Follower, _talonRightA.getDeviceID());
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

    private void setPID(TalonSRX talon, int slot, PID pid)
    {
        talon.config_kP(slot, pid.p, TALON_TIMEOUT);
        talon.config_kI(slot, pid.i, TALON_TIMEOUT);
        talon.config_kD(slot, pid.d, TALON_TIMEOUT);
        talon.config_kF(slot, pid.f, TALON_TIMEOUT);
    }

    private void setNominalAndPeakOutputs(double nominal, double peak)
    {
        _talonLeftA.configNominalOutputForward(nominal, TALON_TIMEOUT);
        _talonLeftA.configPeakOutputForward(peak, TALON_TIMEOUT);
        _talonLeftA.configNominalOutputReverse(-nominal, TALON_TIMEOUT);
        _talonLeftA.configPeakOutputReverse(-peak, TALON_TIMEOUT);

        _talonRightA.configNominalOutputForward(nominal, TALON_TIMEOUT);
        _talonRightA.configPeakOutputForward(peak, TALON_TIMEOUT);
        _talonRightA.configNominalOutputReverse(-nominal, TALON_TIMEOUT);
        _talonRightA.configPeakOutputReverse(-peak, TALON_TIMEOUT);
    }

    private void setLeftRightMotorOutputs(double left, double right)
    {
        _talonLeftA.set(ControlMode.PercentOutput, left);
        _talonLeftB.set(ControlMode.Follower, _talonLeftA.getDeviceID());
        _talonRightA.set(ControlMode.PercentOutput, right);
        _talonRightB.set(ControlMode.Follower, _talonRightA.getDeviceID());
    }

    private int distanceToCounts(double distance)
    {
        return (int)(distance / (CPR * WHEEL_DIAMETER * Math.PI));
    }

    private double applyDeadband(double val, double deadband) {
        return (Math.abs(val) < deadband) ? 0 : val;
    }
}
