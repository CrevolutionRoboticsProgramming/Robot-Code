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
    private enum DriveControlMode
    {
        BASIC,
        FIELD_CENTRIC
    }

    private static SwerveDrive mInstance = new SwerveDrive();
    private DriveControlMode mDriveControlMode = DriveControlMode.FIELD_CENTRIC;
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
            ArrayList<Vector2d> swerveMovementVectors;
            double[] perpendicularAngles = new double[4];

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
                robotCenter = new Vector2d(mConstants.dt_width / 2, mConstants.dt_length / 2);

                perpendicularAngles[0] = Math.atan2(Vector2d.sub(robotCenter, mConstants.dt_topLeftPosition).y, Vector2d.sub(robotCenter, mConstants.dt_topLeftPosition).x) - (Math.PI / 2);
                perpendicularAngles[1] = Math.atan2(Vector2d.sub(robotCenter, mConstants.dt_topRightPosition).y, Vector2d.sub(robotCenter, mConstants.dt_topRightPosition).x) - (Math.PI / 2);
                perpendicularAngles[2] = Math.atan2(Vector2d.sub(robotCenter, mConstants.dt_bottomLeftPosition).y, Vector2d.sub(robotCenter, mConstants.dt_bottomLeftPosition).x) - (Math.PI / 2);
                perpendicularAngles[3] = Math.atan2(Vector2d.sub(robotCenter, mConstants.dt_bottomRightPosition).y, Vector2d.sub(robotCenter, mConstants.dt_bottomRightPosition).x) - (Math.PI / 2);

                for (WPI_TalonSRX talon : mSwivelMotors)
                {
                    talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, mConstants.talonTimeout);
                }
                return false;
            }

            @Override
            public void update()
            {
                swerveMovementVectors = new ArrayList<>();
                Vector2d movement = new Vector2d(mController.get(Axis.AxisID.LEFT_X), mController.get(Axis.AxisID.LEFT_Y));
                double rotationMagnitude = mController.get(Axis.AxisID.RIGHT_X);

                swerveMovementVectors.add(Vector2d.add(movement, new Vector2d((float) (rotationMagnitude * Math.cos(perpendicularAngles[0])), (float) (rotationMagnitude * Math.sin(perpendicularAngles[0])))));
                swerveMovementVectors.add(Vector2d.add(movement, new Vector2d((float) (rotationMagnitude * Math.cos(perpendicularAngles[1])), (float) (rotationMagnitude * Math.sin(perpendicularAngles[1])))));
                swerveMovementVectors.add(Vector2d.add(movement, new Vector2d((float) (rotationMagnitude * Math.cos(perpendicularAngles[2])), (float) (rotationMagnitude * Math.sin(perpendicularAngles[2])))));
                swerveMovementVectors.add(Vector2d.add(movement, new Vector2d((float) (rotationMagnitude * Math.cos(perpendicularAngles[3])), (float) (rotationMagnitude * Math.sin(perpendicularAngles[3])))));

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
                        largestMagnitude = Math.sqrt(Math.pow(vector.x, 2) + Math.pow(vector.y, 2));
                    }
                }

                double multiplier = largestMagnitude > 1 ? 1 / largestMagnitude : largestMagnitude;

                for (int i = 0; i < swerveMovementVectors.size(); ++i)
                {
                    swerveMovementVectors.set(i, new Vector2d((float) (swerveMovementVectors.get(i).x * multiplier), (float) (swerveMovementVectors.get(i).y * multiplier)));
                }

                turnToAngle(mTopLeftSwivel, Math.atan2(swerveMovementVectors.get(0).y, swerveMovementVectors.get(0).x) * 180 / Math.PI);
                turnToAngle(mTopRightSwivel, Math.atan2(swerveMovementVectors.get(1).y, swerveMovementVectors.get(1).x) * 180 / Math.PI);
                turnToAngle(mBottomLeftSwivel, Math.atan2(swerveMovementVectors.get(2).y, swerveMovementVectors.get(2).x) * 180 / Math.PI);
                turnToAngle(mBottomRightSwivel, Math.atan2(swerveMovementVectors.get(3).y, swerveMovementVectors.get(3).x) * 180 / Math.PI);
            }

            @Override
            public void stop()
            {

            }
        };
    }

    public void turnToAngle(WPI_TalonSRX talon, double angle)
    {
        double rotationsCompleted = talon.getSelectedSensorPosition(0) % mConstants.dt_countsPerSwerveRotation;
        double currentAngle = (talon.getSelectedSensorPosition(0) - (rotationsCompleted * mConstants.dt_countsPerSwerveRotation)) / mConstants.dt_countsPerSwerveRotation * 360;
        double counts = angle / 360 * mConstants.dt_countsPerSwerveRotation;
        talon.set(ControlMode.Position, talon.getSelectedSensorPosition(0) + counts);
    }
}
