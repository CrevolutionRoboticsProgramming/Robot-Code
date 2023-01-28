package org.frc2851.crevolib;

import badlog.lib.BadLog;
import badlog.lib.DataInferMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.frc2851.crevolib.auton.Auton;
import org.frc2851.crevolib.auton.AutonExecutor;
import org.frc2851.crevolib.motion.InvalidMotionProfileException;
import org.frc2851.crevolib.motion.MotionProfile;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.crevolib.subsystem.SubsystemManager;
import org.frc2851.crevolib.utilities.FileUtil;
import org.frc2851.crevolib.utilities.Logger;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

@Deprecated
public class CrevoBot extends TimedRobot
{
    private static final String MOTION_PROFILE_DIR = "/home/lvuser/motion/";

    private AutonExecutor mAutonExecutor = new AutonExecutor();
    private SendableChooser<Auton> mAutonSelector = new SendableChooser<>();
    private SubsystemManager mSubManager = SubsystemManager.getInstance();

    private static HashMap<String, MotionProfile> mMotionProfiles = new HashMap<>();

    private BadLog mBadLog;
    private long mStartTimeMs;
    private long mLastLog, mCurrentTimeMs;

    private static boolean mIsEnabled = false;
    private boolean mBadLogEnabled = true;

    /**
     * Adds a subsystem to the robots routine. Also adds it to the logger.
     *
     * @param subsystem Subsystem to be added
     */
    protected void addSubsystem(Subsystem subsystem)
    {
        mSubManager.addSubsystem(subsystem);
    }

    /**
     * Adds an autonomous phase into the robot's routine. It will also add it into the logger
     *
     * @param auton
     */
    protected void addAuton(Auton auton)
    {
        Logger.println("Registered Auton: " + auton.getName(), Logger.LogLevel.DEBUG);
        mAutonSelector.addOption(auton.getName(), auton);
    }

    protected CrevoBot()
    {
        if (mBadLogEnabled)
            mBadLog = BadLog.init("/home/lvuser/log.bag");
    }

    @Override
    public final void robotInit()
    {
        mStartTimeMs = System.nanoTime();
        mLastLog = System.currentTimeMillis();

        Logger.start();
        Logger.println("Robot Init", Logger.LogLevel.DEBUG);
        BadLog.createTopic("Match Time", "s", () -> DriverStation.getInstance().getMatchTime());
        BadLog.createTopicSubscriber("Time", "s", DataInferMode.DEFAULT, "hide", "delta", "xaxis");

        ArrayList<File> files = FileUtil.getFiles(MOTION_PROFILE_DIR, true);
        for (File f : files)
        {
            String name = f.getName().split("\\.")[0];
            try
            {
                mMotionProfiles.put(name, new MotionProfile(f));
            } catch (InvalidMotionProfileException ignored)
            {
            }
        }

        mSubManager.start();

        if (mBadLogEnabled)
            mBadLog.finishInitialization();
    }

    @Override
    public final void autonomousInit()
    {
        Logger.println("Autonomous Init", Logger.LogLevel.DEBUG);
        mAutonExecutor.setAuton(mAutonSelector.getSelected());
        mAutonExecutor.start();
    }

    @Override
    public void teleopInit()
    {
        Logger.println("Teleop Init", Logger.LogLevel.DEBUG);
    }

    @Override
    public final void disabledInit()
    {
        Logger.println("Disabled Init", Logger.LogLevel.DEBUG);
        mAutonExecutor.stop();
        mSubManager.stopAllSubsystems();
        mSubManager.restartDefaultCommands();
    }

    @Override
    public final void robotPeriodic()
    {
        periodic();
    }

    @Override
    public final void disabledPeriodic()
    {
        periodic();
    }

    @Override
    public final void teleopPeriodic()
    {
        periodic();
    }

    @Override
    public final void autonomousPeriodic()
    {
        periodic();
    }

    /**
     * Returns a motion profile fetched from the motion profile directory.
     *
     * @param name The full name of the file, excluding file extensions
     * @return The selected motion profile. If the motion profile specified does not exist,
     * the function returns {@code null}.
     */
    public static MotionProfile getMotionProfile(String name)
    {
        return mMotionProfiles.getOrDefault(name, null);
    }

    private void periodic()
    {
        if (mBadLogEnabled)
        {
            double time = ((double) (System.nanoTime() - mStartTimeMs) / 1000000000d);
            BadLog.publish("Time", time);

            mBadLog.updateTopics();

            mCurrentTimeMs = System.currentTimeMillis();
            if (!this.isDisabled() || (mCurrentTimeMs - mLastLog >= 1000))
            {
                mLastLog = System.currentTimeMillis();
                mBadLog.log();
            }
        }

        mIsEnabled = isEnabled();
    }

    public static boolean isRunning()
    {
        return mIsEnabled;
    }

    public void enableBadLog(boolean enable)
    {
        mBadLogEnabled = enable;
    }
}
