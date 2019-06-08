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
import org.frc2851.crevolib.utilities.Vector2D;

import java.util.ArrayList;
import java.util.HashMap;

public class SwerveDrive extends Subsystem
{
    private static SwerveDrive mInstance = new SwerveDrive();
    private SwerveModule mTopLeftModule, mTopRightModule, mBottomLeftModule, mBottomRightModule;
    private ArrayList<SwerveModule> mSwerveModules = new ArrayList<>();
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

        try
        {
            mTopLeftModule = new SwerveModule(TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_topLeftDrive), TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_topLeftSwivel), mConstants.dt_topLeftPosition);
            mTopRightModule = new SwerveModule(TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_topRightDrive), TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_topRightSwivel), mConstants.dt_topRightPosition);
            mBottomLeftModule = new SwerveModule(TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_bottomLeftDrive), TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_bottomLeftSwivel), mConstants.dt_bottomLeftPosition);
            mBottomRightModule = new SwerveModule(TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_bottomRightDrive), TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_bottomRightSwivel), mConstants.dt_bottomRightPosition);

            for (SwerveModule module : mSwerveModules)
            {
                module.getSwivelTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
            }
        } catch (TalonCommunicationErrorException e)
        {
            log("Could not initialize motor, swerve drive init failed! Port: " + e.getPortNumber(), Logger.LogLevel.ERROR);
            return false;
        }

        mSwerveModules.add(mTopLeftModule);
        mSwerveModules.add(mTopRightModule);
        mSwerveModules.add(mBottomLeftModule);
        mSwerveModules.add(mBottomRightModule);

        for (SwerveModule module : mSwerveModules)
        {
            TalonSRXFactory.configurePIDF(module.getSwivelTalon(), 0, mSwivelPID);
            module.getSwivelTalon().selectProfileSlot(0, 0);
            module.getSwivelTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, mConstants.talonTimeout);
        }

        mPigeon = new PigeonIMU(0);

        return true;
    }

    @Override
    public Command getDefaultCommand()
    {
        return new Command()
        {
            ArrayList<Vector2D> swerveMovementVectors;

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
                Vector2D movement = new Vector2D(mController.get(Axis.AxisID.LEFT_X), mController.get(Axis.AxisID.LEFT_Y));
                double rotationMagnitude = mController.get(Axis.AxisID.RIGHT_X);

                /*
                In these lines we:
                1. Change the signs of the vector representing movement to accommodate its orientation (if it made backwards forwards to save time rotating)
                2. Create a new vector with:
                    a. The x-value as the x-component of the rotation vector
                    b. The y-value as the y-component of the rotation vector
                        i. This is where we use the perpendicular angle of the module; that's what rotates the magnitude and makes it a vector
                3. Add the vectors to get the total vector representing our final movement
                 */
                for (SwerveModule module : mSwerveModules)
                {
                    swerveMovementVectors.add(Vector2D.add(new Vector2D(movement.x, movement.y),
                            new Vector2D((rotationMagnitude * Math.cos(module.getPerpendicularAngle())), (rotationMagnitude * Math.sin(module.getPerpendicularAngle())))));
                }

                // If the largest magnitude is greater than one (which we can't use as a magnitude), set the multiplier to reduce
                // the magnitude of all the vectors by the fraction it takes to reduce the largest magnitude to one
                double largestMagnitude = Math.max(Math.max(swerveMovementVectors.get(0).magnitude(), swerveMovementVectors.get(1).magnitude()), Math.max(swerveMovementVectors.get(2).magnitude(), swerveMovementVectors.get(3).magnitude()));
                if (largestMagnitude > 1.0) {
                    double multiplier = 1 / largestMagnitude;
                    for (int i = 0; i < swerveMovementVectors.size(); ++i)
                        swerveMovementVectors.set(i, new Vector2D((swerveMovementVectors.get(i).x * multiplier), (swerveMovementVectors.get(i).y * multiplier)));
                }

                // Converts the angle to degrees for easier understanding. All the other angles were in radians because the Math trig functions use rads
                for (int i = 0; i < mSwerveModules.size(); ++i)
                {
                    turnToAngle(mSwerveModules.get(i), Math.atan2(swerveMovementVectors.get(i).y, swerveMovementVectors.get(i).x) * 180 / Math.PI - 90);
                    mSwerveModules.get(i).getDriveTalon().set(swerveMovementVectors.get(i).magnitude());
                }
            }

            @Override
            public void stop()
            {
                reset();
            }

            private void turnToAngle(SwerveModule module, double angle)
            {
                double counts = module.getSwivelTalon().getSelectedSensorPosition(0);

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
                if (module.getSwivelTalon().getSelectedSensorPosition(0) < 0)
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
                    module.getSwivelTalon().set(0);
                    module.setStopped(true);
                    if ((!module.getLastStopped() || target != module.getLastTarget()) && (smallestDifference.equals("To Opposite Angle") || smallestDifference.equals("To Opposite Angle Over Gap")))
                    {
                        module.getSwivelTalon().setInverted(!module.getSwivelTalon().getInverted());
                        module.getDriveTalon().setInverted(!module.getDriveTalon().getInverted());
                    }
                } else
                {
                    module.getSwivelTalon().set(ControlMode.Position, module.getSwivelTalon().getSelectedSensorPosition(0) + differences.get(smallestDifference));
                    module.setStopped(false);
                }

                module.setLastStopped(module.isStopped());
                module.setLastTarget(target);
            }
        };
    }

    private void reset()
    {
        for (SwerveModule module : mSwerveModules)
        {
            module.getDriveTalon().set(ControlMode.PercentOutput, 0);
            module.getSwivelTalon().set(ControlMode.PercentOutput, 0);
        }

        resetSensors();
    }

    private void resetSensors()
    {
        for (SwerveModule module : mSwerveModules)
        {
            module.getSwivelTalon().setSelectedSensorPosition(0);
        }
        mPigeon.setYaw(0);
    }
}

class SwerveModule
{
    private WPI_TalonSRX mDriveTalon, mSwivelTalon;
    private boolean mStopped = false;
    private boolean mLastStopped = false;
    private double mLastTarget = 0.0;
    private double mPerpendicularAngle;

    private Constants mConstants;

    public SwerveModule(WPI_TalonSRX driveTalon, WPI_TalonSRX swivelTalon, Vector2D modulePosition)
    {
        mConstants = Constants.getInstance();
        mDriveTalon = driveTalon;
        mSwivelTalon = swivelTalon;

        /* Calculates the angle the module would have to face to be perpendicular to the center of the robot
            Taking the tangent of the difference between the module's position and the robot's center's y-value and that of its x-value
            gives us the module's angle in relation to the center of the robot. Subtracting 90 degrees (or PI/2 radians) gives us the
            perpendicular angle. We use this later to compute rotation vectors.
            atan2 takes into account the quadrant of the angle, unlike atan
            */
        mPerpendicularAngle = Math.atan2(Vector2D.sub(mConstants.robotCenter, modulePosition).y, Vector2D.sub(mConstants.robotCenter, modulePosition).x) - (Math.PI / 2);
    }

    public WPI_TalonSRX getDriveTalon()
    {
        return mDriveTalon;
    }

    public WPI_TalonSRX getSwivelTalon()
    {
        return mSwivelTalon;
    }

    public double getPerpendicularAngle()
    {
        return mPerpendicularAngle;
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