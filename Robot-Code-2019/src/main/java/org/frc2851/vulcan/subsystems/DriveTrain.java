package org.frc2851.vulcan.subsystems;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import org.frc2851.crevolib.io.Axis;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.motion.MotionProfileExecutor;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.vulcan.Robot;

public class DriveTrain extends Subsystem
{
    // Constants
    private final double CPR = 4096;
    private final double WHEEL_DIAMETER = 1.0d / 3.0d;
    private final double DEFAULT_PEAK_OUT = 1;
    private final double DEFAULT_NOMINAL_OUT = 0;
    private final int TALON_TIMEOUT = 0;

    // Motion Magic info
    // Units (Average Counts per 100ms)
    // TODO: Experimentally determine max vel and acc
    private final int MOTION_MAGIC_MAX_VEL = 15000;
    private final int MOTION_MAGIC_MAX_ACC = 3000;

    private final NeutralMode TALON_NEUTRAL_MODE = NeutralMode.Brake;

    // Members
    private WPI_TalonSRX _talonLeftA, _talonLeftB, _talonRightA, _talonRightB;
    private Controller _controller = Robot.driver;

    // Singleton
    private static DriveTrain _instance = new DriveTrain();
    private DriveTrain() { super("DriveTrain"); }
    public static DriveTrain getInstance() { return _instance; }

    @Override
    protected void init()
    {
        _talonLeftA = new WPI_TalonSRX(6);
        _talonLeftB = new WPI_TalonSRX(4);
        _talonRightA = new WPI_TalonSRX(17);
        _talonRightB = new WPI_TalonSRX(10);

        // Motor Controller Configuration
        _talonLeftA.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TALON_TIMEOUT);
        _talonLeftA.setSelectedSensorPosition(0, 0, TALON_TIMEOUT);
        _talonRightA.setSelectedSensorPosition(0, 0, TALON_TIMEOUT);
        _talonRightA.setSensorPhase(true);

        setNominalAndPeakOutputs(DEFAULT_NOMINAL_OUT, DEFAULT_PEAK_OUT);

        _talonLeftB.set(ControlMode.Follower, _talonLeftA.getDeviceID());
        _talonRightB.set(ControlMode.Follower, _talonRightA.getDeviceID());

        // Controller config
        _controller.config(Axis.AxisID.LEFT_Y); // Throttle
        _controller.config(Axis.AxisID.RIGHT_X); // Turn
        _controller.config(Button.ButtonID.RIGHT_BUMPER, Button.ButtonMode.TOGGLE); // Curvature Toggle

        reset(); // Probably unnecessary. Worth the lost cycles for certainty.
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
            final double DEADBAND = 0.15;

            @Override
            public String getName() { return "Teleop"; }

            @Override
            public boolean isFinished() { return false; }

            @Override
            public void init()
            {
                reset();
                robotDrive.tankDrive(0, 0);
                robotDrive.setSafetyEnabled(false);
            }

            @Override
            public void update()
            {
                double throttle = _controller.get(Axis.AxisID.LEFT_Y, null);
                double turn = _controller.get(Axis.AxisID.RIGHT_X, null) * TURN_MULT;
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
            public void init() {

            }

            @Override
            public void update() {

            }

            @Override
            public void stop() {

            }
        };
    }

    public Command turnToAngleEncoder(double angle, double power)
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
            public void init() {

            }

            @Override
            public void update() {

            }

            @Override
            public void stop() {

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
            public void init() {
                timer.start();
            }

            @Override
            public void update() {
                setLeftRightMotorOutputs(leftPower, rightPower);
            }

            @Override
            public void stop() {
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
            public void init()
            {
                reset();
                setPID(0.2, 0, 0, 0);
                setNominalAndPeakOutputs(DEFAULT_NOMINAL_OUT, maxPower);

                _talonLeftA.set(ControlMode.Position, counts);
                _talonRightA.set(ControlMode.Position, counts);
                _talonLeftB.set(ControlMode.Follower, _talonLeftA.getDeviceID());
                _talonRightB.set(ControlMode.Follower, _talonRightA.getDeviceID());
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

    public Command driveDistanceMotionMagic(double distance, double maxPower)
    {
        return new Command()
        {
            boolean isFinished = false;
            int counts = distanceToCounts(distance);
            @Override
            public String getName() { return "DriveDistanceMM[\" + distance + \" feet, \" + power + \"]"; }

            @Override
            public boolean isFinished()
            {
                return isFinished;
            }

            @Override
            public void init()
            {
                reset();
                setPID(0.2, 0, 0, 0);
                setNominalAndPeakOutputs(DEFAULT_NOMINAL_OUT, maxPower);

                _talonLeftA.configMotionCruiseVelocity(MOTION_MAGIC_MAX_VEL, TALON_TIMEOUT);
                _talonRightA.configMotionCruiseVelocity(MOTION_MAGIC_MAX_VEL, TALON_TIMEOUT);
                _talonLeftA.configMotionAcceleration(MOTION_MAGIC_MAX_ACC, TALON_TIMEOUT);
                _talonRightA.configMotionAcceleration(MOTION_MAGIC_MAX_ACC, TALON_TIMEOUT);

                // TODO: Confirm Correct Motion Magic Usage
                _talonLeftA.set(ControlMode.MotionMagic, counts);
                _talonLeftB.set(ControlMode.Follower, _talonLeftA.getDeviceID());
                _talonRightA.set(ControlMode.MotionMagic, counts);
                _talonRightB.set(ControlMode.Follower, _talonRightA.getDeviceID());
            }

            @Override
            public void update()
            {
                // Will finish when the motor is within 1/4 of a rotation
                if (Math.abs(counts - _talonLeftA.getSelectedSensorPosition(0)) < 1024) isFinished = true;
            }

            @Override
            public void stop()
            {
                reset();
            }
        };
    }

    public Command runMotionProfile(String leftProfile, String rightProfile)
    {
        return new Command()
        {
            MotionProfileExecutor left, right;

            @Override
            public String getName() { return "MotionProfile[" + leftProfile + ", " + rightProfile + "]"; }

            @Override
            public boolean isFinished()
            {
                SetValueMotionProfile leftSet, rightSet;
                leftSet = left.getSetValue();
                rightSet = right.getSetValue();

                return leftSet == SetValueMotionProfile.Hold && rightSet == SetValueMotionProfile.Hold;
            }

            @Override
            public void init()
            {
//                left = new MotionProfileExecutor(leftProfile, _talonLeftA, CPR / (WHEEL_DIAMETER * Math.PI));
//                right = new MotionProfileExecutor(rightProfile, _talonRightA, CPR / (WHEEL_DIAMETER * Math.PI));

                left = new MotionProfileExecutor(Robot.getMotionProfile(leftProfile), _talonLeftA);
                right = new MotionProfileExecutor(Robot.getMotionProfile(rightProfile), _talonRightA);

                left.start();
                right.start();
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

    private void setPID(double p, double i, double d, double f)
    {
        _talonLeftA.config_kP(0, p, 0);
        _talonLeftA.config_kI(0, i, 0);
        _talonLeftA.config_kD(0, d, 0);
        _talonLeftA.config_kF(0, f, 0);

        _talonRightA.config_kP(0, p, 0);
        _talonRightA.config_kI(0, i, 0);
        _talonRightA.config_kD(0, d, 0);
        _talonRightA.config_kF(0, f, 0);
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
}
