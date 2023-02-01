package org.frc2851.crevolib.motion;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import org.frc2851.crevolib.utilities.Logger;

import java.util.ArrayList;

/**
 * MotionProfileExecutor: A class which takes in a file and a talon and executes a motion profile
 * This class takes in a pathfinder csv file, converts it to a format readable by a TalonSRX, and
 * handles running the motion profile on the Talon.
 */
public class MotionProfileExecutor
{
    private class InvalidMotionProfileException extends Exception
    {
    }

    private enum State
    {
        DISABLED, LOADING, RUNNING, COMPLETE
    }

    private TalonSRX mTalon;
    private boolean mStart = false, mProfileComplete = false;
    private Notifier mNotifier;

    private SetValueMotionProfile mSetValue = SetValueMotionProfile.Disable;
    private MotionProfileStatus mStatus = new MotionProfileStatus();
    private State mState = State.DISABLED;
    private final int MIN_POINTS = 10;

    private MotionProfile mProfile = null;
    private final boolean USE_ARC;

    /**
     * Creates the MotionProfileExecutor from a given MotionProfile and Talon
     *
     * @param profile The profile
     * @param talon   The talon
     * @param useArc  Tells the executor to use heading correction
     */
    public MotionProfileExecutor(MotionProfile profile, TalonSRX talon, boolean useArc) throws NullPointerException
    {
        if (profile == null)
        {
            Logger.println("Null Motion Profile", Logger.LogLevel.ERROR);
            throw new NullPointerException();
        }

        mTalon = talon;
        mProfile = profile;
        mNotifier = new Notifier(() -> mTalon.processMotionProfileBuffer());
        mNotifier.startPeriodic(0.005);
        USE_ARC = useArc;
    }

    /**
     * Updates that state of the Talon and handles streaming points.
     */
    public void update()
    {
        switch (mState)
        {
            case DISABLED:
            {
                if (mStart)
                {
                    log("Starting profile", Logger.LogLevel.DEBUG);
                    mState = State.LOADING;
                    mStart = false;
                } else
                {
                    reset();
                }
                break;
            }

            case LOADING:
            {
                log("Filling buffer", Logger.LogLevel.DEBUG);
                mSetValue = SetValueMotionProfile.Disable;
                mState = State.RUNNING;
                try
                {
                    fillBuffer();
                } catch (InvalidMotionProfileException e)
                {
                    mState = State.DISABLED;
                    log("Could not run motion profile. Failed to fill buffer.", Logger.LogLevel.ERROR);
                }
                break;
            }

            case RUNNING:
            {
                if (mStatus.btmBufferCnt > MIN_POINTS)
                {
                    log("Enabling motion profile", Logger.LogLevel.DEBUG);
                    mSetValue = SetValueMotionProfile.Enable;
                    mState = State.COMPLETE;
                }
                break;
            }

            case COMPLETE:
            {
                // TODO: What if mp stuck? Needs testing.
                if (mStatus.isLast && mStatus.activePointValid)
                {
                    log("Profile complete", Logger.LogLevel.DEBUG);
                    mSetValue = SetValueMotionProfile.Hold;
                    mProfileComplete = true;
                }
                break;
            }
        }
        mTalon.getMotionProfileStatus(mStatus);
    }

    /**
     * Fills the Talon's buffer of Motion Profile points with those stored
     *
     * @throws InvalidMotionProfileException
     */
    private void fillBuffer() throws InvalidMotionProfileException
    {
        ArrayList<MotionProfilePoint> points = mProfile.getPoints();

        if (mProfile.getSize() > 2048)
        {
            DriverStation.reportError("Motion Profile Size Exceeds 2048 Points", false);
            throw new InvalidMotionProfileException();
        }

        for (int i = 0; i < points.size(); i++)
        {
            TrajectoryPoint point = new TrajectoryPoint();
            point.position = points.get(i).pos;
            point.velocity = points.get(i).vel;
            point.timeDur = points.get(i).dt;
            point.profileSlotSelect0 = 0;
            point.profileSlotSelect1 = 0;
            point.headingDeg = (USE_ARC) ? points.get(i).heading : 0;
            point.zeroPos = i == 0;
            point.isLastPoint = (i + 1) == points.size();
            if (point.isLastPoint) System.out.println("Last Point[" + i + "]");

            mTalon.pushMotionProfileTrajectory(point);
        }
    }

    /**
     * Resets the talon and all executor flags
     */
    public void reset()
    {
        mTalon.clearMotionProfileTrajectories();
        mSetValue = SetValueMotionProfile.Disable;
        mState = State.DISABLED;
        mStart = false;
        mProfileComplete = false;
    }

    /**
     * Starts the executor
     */
    public void start()
    {
        mStart = true;
        mProfileComplete = false;
    }

    /**
     * Returns true if the profile has run to completion
     *
     * @return {@code true} if profile complete
     */
    public boolean isProfileComplete()
    {
        return mProfileComplete;
    }

    /**
     * Gets the SetValue for the talon that is given by the executor.
     * <p>
     * Note: The executor does <b>NOT</b> set the talon value. This needs to be done in the subsystem.
     *
     * @return The SetValue
     */
    public SetValueMotionProfile getSetValue()
    {
        return mSetValue;
    }

    private void log(String message, Logger.LogLevel level)
    {
        Logger.println("[MP: " + mProfile.getName() + "] " + message, level);
    }
}
