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

public class SwerveDrive extends Subsystem
{
    private static SwerveDrive mInstance = new SwerveDrive();
    private WPI_TalonSRX mTopLeftDrive, mTopRightDrive, mBottomLeftDrive, mBottomRightDrive,
        mTopLeftSwivel, mTopRightSwivel, mBottomLeftSwivel, mBottomRightSwivel;
    private ArrayList<WPI_TalonSRX> mDriveMotors = new ArrayList<>(), mSwivelMotors = new ArrayList<>();
    private PID swivelPID;
    private PigeonIMU mPigeon;
    private Controller mController = Constants.driver;
    private Constants mConstants = Constants.getInstance();

    private SwerveDrive() { super("Swerve Drive"); }

    public static SwerveDrive getInstance()
    {
        if (mInstance == null) mInstance = new SwerveDrive();
        return mInstance;
    }

    @Override
    protected boolean init()
    {
        swivelPID = new PID(0, 0, 0, 0);

        mController.config(Axis.AxisID.LEFT_X);
        mController.config(Axis.AxisID.LEFT_Y);
        mController.config(Axis.AxisID.RIGHT_X);

        mDriveMotors.add(mTopLeftDrive);
        mDriveMotors.add(mTopRightDrive);
        mDriveMotors.add(mBottomLeftDrive);
        mDriveMotors.add(mBottomRightDrive);
        mSwivelMotors.add(mTopLeftSwivel);
        mSwivelMotors.add(mTopRightSwivel);
        mSwivelMotors.add(mBottomLeftSwivel);
        mSwivelMotors.add(mBottomRightSwivel);

        try
        {
            mTopLeftDrive = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_topLeftDrive);
            mTopRightDrive = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_topRightDrive);
            mBottomLeftDrive = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_bottomLeftDrive);
            mBottomRightDrive = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_bottomRightDrive);
            mTopLeftSwivel = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_topLeftSwivel);
            mTopRightSwivel = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_topRightSwivel);
            mBottomLeftSwivel = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_bottomLeftSwivel);
            mBottomRightSwivel = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.dt_bottomRightSwivel);

            for (WPI_TalonSRX talon : mSwivelMotors)
            {
                talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
            }
        } catch (TalonCommunicationErrorException e)
        {
            log("Could not initialize motor, swerve drive init failed! Port: " + e.getPortNumber(), Logger.LogLevel.ERROR);
            return false;
        }

        for (WPI_TalonSRX talon : mSwivelMotors)
        {
            TalonSRXFactory.configurePIDF(talon, 0, swivelPID);
        }

        mPigeon = new PigeonIMU(0);

        return true;
    }

    @Override
    public Command getDefaultCommand()
    {
        return new Command()
        {
            Vector2d robotCenter;
            
            // Yes, I'm that lazy. All of these values can be stored as individual variables, but I didn't feel like it
            ArrayList<Vector2d> swerveMovementVectors;
            double[] perpendicularAngles = new double[4];
            int[] driveMultipliers = new int[4];
            
            // This actually has to be an array so it can be passed into getYawPitchRoll() later
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

                robotCenter = new Vector2d(mConstants.dt_width / 2, mConstants.dt_length / 2);

                // Calculates the angles each wheel would have to face to be perpendicular to the center of the robot
                // Taking the tangent of the difference between the wheel's position and the robot's center's y-value and that of its x-value
                // gives us the wheel's angle in relation to the center of the robot. Subtracting 90 degrees (or PI/2 radians) gives us the
                // perpendicular angle. We use this later to compute rotation vectors
                // atan2 takes into account the quadrant of the angle, unlike atan
                perpendicularAngles[0] = Math.atan2(Vector2d.sub(robotCenter, mConstants.dt_topLeftPosition).y, Vector2d.sub(robotCenter, mConstants.dt_topLeftPosition).x) - (Math.PI / 2);
                perpendicularAngles[1] = Math.atan2(Vector2d.sub(robotCenter, mConstants.dt_topRightPosition).y, Vector2d.sub(robotCenter, mConstants.dt_topRightPosition).x) - (Math.PI / 2);
                perpendicularAngles[2] = Math.atan2(Vector2d.sub(robotCenter, mConstants.dt_bottomLeftPosition).y, Vector2d.sub(robotCenter, mConstants.dt_bottomLeftPosition).x) - (Math.PI / 2);
                perpendicularAngles[3] = Math.atan2(Vector2d.sub(robotCenter, mConstants.dt_bottomRightPosition).y, Vector2d.sub(robotCenter, mConstants.dt_bottomRightPosition).x) - (Math.PI / 2);

                // Makes sure all values we pull exist
                for (int i = 0; i < 4; ++i)
                {
                    driveMultipliers[i] = 0;
                }

                for (WPI_TalonSRX talon : mSwivelMotors)
                {
                    talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, mConstants.talonTimeout);
                }
                
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
                swerveMovementVectors.add(Vector2d.add(new Vector2d(driveMultipliers[0] * movement.x, driveMultipliers[0] * movement.y), new Vector2d((rotationMagnitude * Math.cos(perpendicularAngles[0])), (rotationMagnitude * Math.sin(perpendicularAngles[0])))));
                swerveMovementVectors.add(Vector2d.add(new Vector2d(driveMultipliers[1] * movement.x, driveMultipliers[1] * movement.y), new Vector2d((rotationMagnitude * Math.cos(perpendicularAngles[1])), (rotationMagnitude * Math.sin(perpendicularAngles[1])))));
                swerveMovementVectors.add(Vector2d.add(new Vector2d(driveMultipliers[2] * movement.x, driveMultipliers[2] * movement.y), new Vector2d((rotationMagnitude * Math.cos(perpendicularAngles[2])), (rotationMagnitude * Math.sin(perpendicularAngles[2])))));
                swerveMovementVectors.add(Vector2d.add(new Vector2d(driveMultipliers[3] * movement.x, driveMultipliers[3] * movement.y), new Vector2d((rotationMagnitude * Math.cos(perpendicularAngles[3])), (rotationMagnitude * Math.sin(perpendicularAngles[3])))));

                double largestMagnitude = 1;
                for (Vector2d vector : swerveMovementVectors)
                {
                    boolean largest = true;
                    for (Vector2d comparisonVector : swerveMovementVectors)
                    {
                        if (vector.magnitude() < comparisonVector.magnitude())
                        {
                            largest = false;
                            break;
                        }
                    }

                    if(largest)
                    {
                        largestMagnitude = vector.magnitude();
                        break;
                    }
                }

                // If the largest magnitude is greater than one (which we can't use as a magnitude), set the multiplier to reduce
                // the magnitude of all the vectors by the fraction it takes to reduce the largest magnitude to one
                double multiplier = largestMagnitude > 1 ? 1 / largestMagnitude : 1;

                for (int i = 0; i < swerveMovementVectors.size(); ++i)
                {
                    swerveMovementVectors.set(i, new Vector2d((swerveMovementVectors.get(i).x * multiplier), (swerveMovementVectors.get(i).y * multiplier)));
                }

                // Converts the angle to degrees for easier understanding. All the other angles were in radians because the Math trig functions use rads
                turnToAngle(mTopLeftSwivel, Math.atan2(swerveMovementVectors.get(0).y, swerveMovementVectors.get(0).x) * 180 / Math.PI, 0);
                turnToAngle(mTopRightSwivel, Math.atan2(swerveMovementVectors.get(1).y, swerveMovementVectors.get(1).x) * 180 / Math.PI, 1);
                turnToAngle(mBottomLeftSwivel, Math.atan2(swerveMovementVectors.get(2).y, swerveMovementVectors.get(2).x) * 180 / Math.PI, 2);
                turnToAngle(mBottomRightSwivel, Math.atan2(swerveMovementVectors.get(3).y, swerveMovementVectors.get(3).x) * 180 / Math.PI, 3);
            }

            @Override
            public void stop()
            {
                reset();
            }

            public void turnToAngle(WPI_TalonSRX talon, double angle, int index)
            {
                // This gives us the counts of the swivel as if rotating it 360 degrees looped its angle back to 0
                double rotationsCompleted = talon.getSelectedSensorPosition(0) % mConstants.dt_countsPerSwerveRotation;
                double absoluteCounts = (Math.abs(talon.getSelectedSensorPosition(0)) - (rotationsCompleted * mConstants.dt_countsPerSwerveRotation)) / mConstants.dt_countsPerSwerveRotation * 360;

                double counts = angle / 360 * mConstants.dt_countsPerSwerveRotation;

                // Puts negative counts in positive terms
                if (absoluteCounts < 0)
                {
                    absoluteCounts = mConstants.dt_countsPerSwerveRotation + absoluteCounts;
                }

                // We had to put the sensor position through Math.abs before, so this fixes absoluteCounts
                if (talon.getSelectedSensorPosition(0) < 0)
                {
                    absoluteCounts = mConstants.dt_countsPerSwerveRotation - absoluteCounts;
                }

                // This enables field-centric driving. It adds the rotation of the robot to the total counts so it knows where to turn
                mPigeon.getYawPitchRoll(ypr);
                absoluteCounts += ypr[0] * mConstants.dt_countsPerSwerveRotation;

                // If it's faster to turn to the angle opposite of the angle we were given and drive backwards, do that thing
                if (counts - absoluteCounts > 180)
                {
                    counts = mConstants.dt_countsPerSwerveRotation - counts;
                    driveMultipliers[index] = -1;
                } else
                {
                    driveMultipliers[index] = 1;
                }

                double difference = counts - absoluteCounts;

                talon.set(ControlMode.Position, talon.getSelectedSensorPosition(0) + difference);
            }
        };
    }

    private void reset()
    {
        for(WPI_TalonSRX talon : mDriveMotors)
        {
            talon.set(ControlMode.PercentOutput, 0);
        }
        for(WPI_TalonSRX talon : mSwivelMotors)
        {
            talon.set(ControlMode.PercentOutput, 0);
        }

        resetSensors();
    }

    private void resetSensors()
    {
        for (WPI_TalonSRX talon : mSwivelMotors)
        {
            talon.setSelectedSensorPosition(0);
        }
        mPigeon.setYaw(0);
    }
}
