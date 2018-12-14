package org.frc2851.crevolib.motion;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.sun.istack.internal.NotNull;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.HLUsageReporting;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Talon;
import org.frc2851.crevolib.Logger;

import java.io.File;
import java.util.Vector;

/**
 * MotionProfileExecutor: A class which takes in a file and a talon and executes a motion profile
 * This class takes in a pathfinder csv file, converts it to a format readable by a TalonSRX, and
 * handles running the motion profile on the Talon.
 *
 * Unit Conversions:
 *  - Pathfinder Unit (Velocity) (f/s)
 *  - CTRE Unit (Velocity) (avg_counts/100ms)
 */

public class MotionProfileExecutor
{
    private class InvalidMotionProfileException extends Exception {}

    private enum State { DISABLED, LOADING, RUNNING, COMPLETE }

    private TalonSRX _talon;
    private boolean _start = false, _profileComplete = false;
    private Notifier _notifier;

    private SetValueMotionProfile _setValue = SetValueMotionProfile.Disable;
    private MotionProfileStatus _status = new MotionProfileStatus();
    private State _state = State.DISABLED;
    private final int MIN_POINTS = 10;

    private MotionProfile _profile = null;
    private boolean useArc = false;

    /**
     * Creates the MotionProfileExecutor from a given MotionProfile and Talon
     * @param profile The profile
     * @param talon The talon
     * @param useArc Tells the executor to use heading correction
     */
    public MotionProfileExecutor(MotionProfile profile, TalonSRX talon, boolean useArc) throws NullPointerException
    {
        if (profile == null) {
            Logger.println("Null Motion Pointer", Logger.LogLevel.ERROR);
            throw new NullPointerException();
        }
        _talon = talon;
        _profile = profile;
        _notifier = new Notifier(() -> _talon.processMotionProfileBuffer());
        _notifier.startPeriodic(0.005);
        this.useArc = useArc;
    }

    /**
     * Creates a MotionProfileExecutor from a given file, talon, and counts per feet value. A motion profile is created
     * from the file and cpf.
     * @param path The path of the file
     * @param talon The talon
     * @param cpf The count per feet value
     */
    public MotionProfileExecutor(String path, TalonSRX talon, int cpf) {
        _talon = talon;
        try {
            _profile = new MotionProfile(new File(path));
        } catch (BadMotionProfileException ignored) {

        }

        _notifier = new Notifier(() -> _talon.processMotionProfileBuffer());
        _notifier.startPeriodic(0.005);
    }

    /**
     *
     */
    public void update()
    {
        switch (_state)
        {
            case DISABLED:
            {
                if (_start) {
                    _state = State.LOADING;
                    _start = false;
                } else {
                    reset();
                }
                break;
            }

            case LOADING:
            {
                System.out.println("Loading");
                _setValue = SetValueMotionProfile.Disable;
                _state = State.RUNNING;
                try {
                    fillBuffer();
                } catch (InvalidMotionProfileException e) {
                    _state = State.DISABLED;
                    DriverStation.reportError("Could not run motion profile. Failed to fill buffer.", false);
                }
                break;
            }

            case RUNNING:
            {
                System.out.println("Running");
                if (_status.btmBufferCnt > MIN_POINTS)
                {
                    _setValue = SetValueMotionProfile.Enable;
                    _state = State.COMPLETE;
                }
                break;
            }

            case COMPLETE:
            {
                // TODO: What if mp stuck? Needs testing.
                if (_status.isLast && _status.activePointValid)
                {
                    _setValue = SetValueMotionProfile.Hold;
                    _profileComplete = true;
                }
                break;
            }
        }
        _talon.getMotionProfileStatus(_status);
    }

    private void fillBuffer() throws InvalidMotionProfileException
    {
        Vector<MotionProfilePoint> points = _profile.getPoints();

        if (_profile.getSize() > 2048)
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
            point.headingDeg = (useArc) ? points.get(i).heading : 0;
            point.zeroPos = i == 0;
            point.isLastPoint = (i + 1) == points.size();
            if (point.isLastPoint) System.out.println("Last Point[" + i + "]");

            _talon.pushMotionProfileTrajectory(point);
            System.out.println("Pushing Point: " + i);
        }

    }

    /**
     * Resets the talon and all executor flags
     */
    public void reset()
    {
        _talon.clearMotionProfileTrajectories();
        _setValue = SetValueMotionProfile.Disable;
        _state = State.DISABLED;
        _start = false;
        _profileComplete = false;
    }

    /**
     * Starts the executor
     */
    public void start()
    {
        _start = true;
        _profileComplete = false;
    }

    /**
     * Returns true if the profile has run to completion
     * @return {@code true} if profile complete
     */
    public boolean isProfileComplete() { return _profileComplete; }

    /**
     * Gets the SetValue for the talon that is given by the executor.
     *
     * Note: The executor does <b>NOT</b> set the talon value. This needs to be done in the subsystem.
     * @return The SetValue
     */
    public SetValueMotionProfile getSetValue() { return _setValue; }
}
