package org.frc2851.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import org.frc2851.crevolib.io.Axis;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.motion.PID;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.crevolib.utilities.Logger;
import org.frc2851.crevolib.utilities.TalonCommunicationErrorException;
import org.frc2851.crevolib.utilities.TalonSRXFactory;
import org.frc2851.crevolib.utilities.Vector2d;

import java.util.ArrayList;
import java.util.HashMap;

public class SwerveDrive extends Subsystem
{
    private static SwerveDrive mInstance = new SwerveDrive();
    private WPI_TalonSRX mTopLeftDrive, mTopRightDrive, mBottomLeftDrive, mBottomRightDrive;
    private SwivelWheel mTopLeftSwivel, mTopRightSwivel, mBottomLeftSwivel, mBottomRightSwivel;
    private ArrayList<WPI_TalonSRX> mDriveMotors = new ArrayList<>();
    private ArrayList<SwivelWheel> mSwivelWheels = new ArrayList<>();
    private PID mSwivelPID;
    private PigeonIMU mPigeon;
    private Controller mController = Constants.driver;
    private Constants mConstants = Constants.getInstance();

    private SwerveDrive()
    {
        super("Swerve Drive");
    }

    public static SwerveDrive getInstance()
    {
        if (mInstance == null) mInstance = new SwerveDrive();
        return mInstance;
    }

    @Override
    protected boolean init()
    {
        mSwivelPID = new PID(0, 0, 0, 0);

        mController.config(Axis.AxisID.LEFT_X);
        mController.config(Axis.AxisID.LEFT_Y);
        mController.config(Axis.AxisID.RIGHT_X);

        mDriveMotors.add(mTopLeftDrive);
        mDriveMotors.add(mTopRightDrive);
        mDriveMotors.add(mBottomLeftDrive);
        mDriveMotors.add(mBottomRightDrive);
        mSwivelWheels.add(mTopLeftSwivel);
        mSwivelWheels.add(mTopRightSwivel);
        mSwivelWheels.add(mBottomLeftSwivel);
        mSwivelWheels.add(mBottomRightSwivel);

        try
        {
            mTopLeftDrive = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_topLeftDrive);
            mTopRightDrive = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_topRightDrive);
            mBottomLeftDrive = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_bottomLeftDrive);
            mBottomRightDrive = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_bottomRightDrive);

            mTopLeftSwivel = new SwivelWheel(TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_topLeftSwivel), mConstants.dt_topLeftPosition);
            mTopRightSwivel = new SwivelWheel(TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_topRightSwivel), mConstants.dt_topRightPosition);
            mBottomLeftSwivel = new SwivelWheel(TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_bottomLeftSwivel), mConstants.dt_bottomLeftPosition);
            mBottomRightSwivel = new SwivelWheel(TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_bottomRightSwivel), mConstants.dt_bottomRightPosition);

            for (SwivelWheel wheel : mSwivelWheels)
            {
                wheel.getTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
            }
        } catch (TalonCommunicationErrorException e)
        {
            log("Could not initialize motor, swerve drive init failed! Port: " + e.getPortNumber(), Logger.LogLevel.ERROR);
            return false;
        }

        for (SwivelWheel wheel : mSwivelWheels)
        {
            TalonSRXFactory.configurePIDF(wheel.getTalon(), 0, mSwivelPID);
            wheel.getTalon().selectProfileSlot(0, 0);
            wheel.getTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, mConstants.talonTimeout);
        }

        mPigeon = new PigeonIMU(0);

        return true;
    }

    @Override
    public Command getDefaultCommand()
    {
        return new Command()
        {
            ArrayList<Vector2d> swerveMovementVectors;

            // This has to be an array so it can be passed into getYawPitchRoll() later
            double[] ypr = new double[3];

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
                swerveMovementVectors = new ArrayList<>();
                Vector2d movement = new Vector2d(mController.get(Axis.AxisID.LEFT_X), mController.get(Axis.AxisID.LEFT_Y));
                double rotationMagnitude = mController.get(Axis.AxisID.RIGHT_X);

                /*
                In these lines we:
                1. Change the signs of the vector representing movement to accommodate its orientation (if it made backwards forwards to save time rotating)
                2. Create a new vector with:
                    a. The x-value as the x-component of the rotation vector
                    b. The y-value as the y-component of the rotation vector
                        i. This is where we use the perpendicular angle of the wheel; that's what rotates the magnitude and makes it a vector
                3. Add the vectors to get the total vector representing our final movement
                 */
                for (SwivelWheel wheel : mSwivelWheels)
                {
                    swerveMovementVectors.add(Vector2d.add(new Vector2d(wheel.getDriveMultiplier() * movement.x, wheel.getDriveMultiplier() * movement.y),
                            new Vector2d((wheel.getDriveMultiplier() * rotationMagnitude * Math.cos(wheel.getPerpendicularAngle())), (wheel.getDriveMultiplier() * rotationMagnitude * Math.sin(wheel.getPerpendicularAngle())))));
                }

                // If the largest magnitude is greater than one (which we can't use as a magnitude), set the multiplier to reduce
                // the magnitude of all the vectors by the fraction it takes to reduce the largest magnitude to one
                double largestMagnitude = Math.max(Math.max(swerveMovementVectors.get(0).magnitude(), swerveMovementVectors.get(1).magnitude()), Math.max(swerveMovementVectors.get(2).magnitude(), swerveMovementVectors.get(3).magnitude()));
                if (largestMagnitude > 1.0) {
                    double multiplier = 1 / largestMagnitude;
                    for (int i = 0; i < swerveMovementVectors.size(); ++i)
                        swerveMovementVectors.set(i, new Vector2d((swerveMovementVectors.get(i).x * multiplier), (swerveMovementVectors.get(i).y * multiplier)));
                }

                // Converts the angle to degrees for easier understanding. All the other angles were in radians because the Math trig functions use rads
                for (int i = 0; i < mDriveMotors.size(); ++i)
                {
                    turnToAngle(mSwivelWheels.get(i), Math.atan2(swerveMovementVectors.get(i).y, swerveMovementVectors.get(i).x) * 180 / Math.PI - 90);
                    mDriveMotors.get(i).set(swerveMovementVectors.get(i).magnitude() * mTopLeftSwivel.getDriveMultiplier());
                }
            }

            @Override
            public void stop()
            {
                reset();
            }

            private void turnToAngle(SwivelWheel wheel, double angle)
            {
                double counts = wheel.getTalon().getSelectedSensorPosition(0);

                // This enables field-centric driving. It adds the rotation of the robot to the total counts so it knows where to turn
                mPigeon.getYawPitchRoll(ypr);
                counts += (ypr[0] - ((int) (ypr[0] / 360) * 360)) / 360 * mConstants.dt_countsPerSwerveRotation;
                // TODO: What's the resolution of a pigeon? Find it and replace 360

                // This gives us the counts of the swivel on a scale from 0 to the total counts per rotation
                int rotationsCompleted = (int) (counts / mConstants.dt_countsPerSwerveRotation);
                counts = Math.abs(counts) - (rotationsCompleted * mConstants.dt_countsPerSwerveRotation);

                // Converts target to ticks and puts it on scale from -360 to 360
                double target = angle / 360 * mConstants.dt_countsPerSwerveRotation;

                // Puts negative counts in positive terms
                if (target < 0) target += mConstants.dt_countsPerSwerveRotation;

                // Puts negative counts in positive terms
                if (counts < 0) counts += mConstants.dt_countsPerSwerveRotation;

                // We had to put the sensor position through Math.abs before, so this fixes absoluteCounts
                if (wheel.getTalon().getSelectedSensorPosition(0) < 0)
                    counts = mConstants.dt_countsPerSwerveRotation - counts;

                double oppositeAngle = target - (mConstants.dt_countsPerSwerveRotation / 2);
                if (oppositeAngle < 0) oppositeAngle += mConstants.dt_countsPerSwerveRotation;

                HashMap<String, Double> differences = new HashMap<>();
                differences.put("Best Case", Math.max(counts, target) - Math.min(counts, target));
                differences.put("Over Gap", Math.min(counts, target) + (mConstants.dt_countsPerSwerveRotation - Math.max(counts, target)));
                differences.put("To Opposite Angle", Math.max(counts, oppositeAngle) - Math.min(counts, oppositeAngle));
                differences.put("To Opposite Angle Over Gap", Math.min(counts, oppositeAngle) + (mConstants.dt_countsPerSwerveRotation - Math.max(counts, oppositeAngle)));

                String smallestDifference = "";
                for (HashMap.Entry<String, Double> pair : differences.entrySet())
                {
                    boolean smallest = true;
                    for (HashMap.Entry<String, Double> comparePair : differences.entrySet())
                    {
                        if (pair.getValue() > comparePair.getValue())
                        {
                            smallest = false;
                            break;
                        }
                    }
                    if (smallest)
                    {
                        smallestDifference = pair.getKey();
                        break;
                    }
                }

                if (counts > target)
                    differences.replace("Best Case", -differences.get("Best Case"));
                if (target > counts)
                    differences.replace("Over Gap", -differences.get("Over Gap"));
                if (counts > oppositeAngle)
                    differences.replace("To Opposite Angle", -differences.get("To Opposite Angle"));
                if (oppositeAngle > counts)
                    differences.replace("To Opposite Angle Over Gap", -differences.get("To Opposite Angle Over Gap"));

                if (Math.abs(differences.get(smallestDifference)) < 8)
                {
                    wheel.getTalon().set(0);
                    wheel.setStopped(true);
                    if ((!wheel.getLastStopped() || target != wheel.getLastTarget()) && (smallestDifference.equals("To Opposite Angle") || smallestDifference.equals("To Opposite Angle Over Gap")))
                    {
                        wheel.setDriveMultiplier(-wheel.getDriveMultiplier());
                    }
                } else
                {
                    wheel.getTalon().set(ControlMode.Position, wheel.getTalon().getSelectedSensorPosition(0) + differences.get(smallestDifference));
                    wheel.setStopped(false);
                }

                wheel.setLastStopped(wheel.isStopped());
                wheel.setLastTarget(target);
            }
        };
    }

    private void reset()
    {
        for (WPI_TalonSRX talon : mDriveMotors)
        {
            talon.set(ControlMode.PercentOutput, 0);
        }
        for (SwivelWheel wheel : mSwivelWheels)
        {
            wheel.getTalon().set(ControlMode.PercentOutput, 0);
        }

        resetSensors();
    }

    private void resetSensors()
    {
        for (SwivelWheel wheel : mSwivelWheels)
        {
            wheel.getTalon().setSelectedSensorPosition(0);
        }
        mPigeon.setYaw(0);
    }
}

class SwivelWheel
{
    private WPI_TalonSRX mTalon;
    private double mDriveMultiplier = 1.0;
    private boolean mStopped = false;
    private boolean mLastStopped = false;
    private double mLastTarget = 0.0;
    private double mPerpendicularAngle;

    private Constants mConstants;

    public SwivelWheel(WPI_TalonSRX talon, Vector2d wheelPosition)
    {
        mConstants = Constants.getInstance();
        mTalon = talon;

        /* Calculates the angle the wheel would have to face to be perpendicular to the center of the robot
            Taking the tangent of the difference between the wheel's position and the robot's center's y-value and that of its x-value
            gives us the wheel's angle in relation to the center of the robot. Subtracting 90 degrees (or PI/2 radians) gives us the
            perpendicular angle. We use this later to compute rotation vectors.
            atan2 takes into account the quadrant of the angle, unlike atan
            */
        mPerpendicularAngle = Math.atan2(Vector2d.sub(mConstants.robotCenter, wheelPosition).y, Vector2d.sub(mConstants.robotCenter, wheelPosition).x) - (Math.PI / 2);
    }

    public WPI_TalonSRX getTalon()
    {
        return mTalon;
    }

    public double getPerpendicularAngle()
    {
        return mPerpendicularAngle;
    }

    public double getDriveMultiplier()
    {
        return mDriveMultiplier;
    }

    public void setDriveMultiplier(double driveMultiplier)
    {
        mDriveMultiplier = driveMultiplier;
    }

    public boolean isStopped()
    {
        return mStopped;
    }

    public void setStopped(boolean stopped)
    {
        mStopped = stopped;
    }

    public boolean getLastStopped()
    {
        return mLastStopped;
    }

    public void setLastStopped(boolean lastStopped)
    {
        mLastStopped = lastStopped;
    }

    public double getLastTarget()
    {
        return mLastTarget;
    }

    public void setLastTarget(double lastTarget)
    {
        mLastTarget = lastTarget;
    }
}