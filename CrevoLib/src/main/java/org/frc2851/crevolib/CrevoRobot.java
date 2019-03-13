package org.frc2851.crevolib;

import badlog.lib.BadLog;
import badlog.lib.DataInferMode;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.frc2851.crevolib.auton.Auton;
import org.frc2851.crevolib.auton.AutonExecutor;
import org.frc2851.crevolib.motion.InvalidMotionProfileException;
import org.frc2851.crevolib.motion.MotionProfile;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.crevolib.subsystem.SubsystemManager;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

public class CrevoRobot extends RobotBase
{
    private static final String MOTION_PROFILE_DIR = "/home/lvuser/motion/";

    private AutonExecutor mAutonExecutor = new AutonExecutor();
    private SendableChooser<Auton> mAutonSelector = new SendableChooser<>();
    private SubsystemManager mSubManager = SubsystemManager.getInstance();

    private static HashMap<String, MotionProfile> mMotionProfiles = new HashMap<>();

    private BadLog mBadLog;
    private long mStartTimeMs;
    private long mLastLog, mCurrentTimeMs;

    private boolean mAutonomousInit = false, mTeleopInit = false, mDisabledInit = false;
    private static boolean mIsEnabled = false;

    private int mPeriod = 5;

    protected CrevoRobot()
    {
        mBadLog = BadLog.init("/home/lvuser/log.bag");
    }

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

    public static MotionProfile getMotionProfile(String name)
    {
        return mMotionProfiles.getOrDefault(name, null);
    }

    public static boolean isRunning()
    {
        return mIsEnabled;
    }

    public void setPeriodMillis(int millis)
    {
        mPeriod = millis;
    }


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

        mBadLog.finishInitialization();
    }

    private void periodic()
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

        mIsEnabled = isEnabled();
    }

    public final void startCompetition()
    {
        robotInit();

        HAL.observeUserProgramStarting();

        // Main Loop
        while (true)
        {
            if (isAutonomous())
            {
                if (!mAutonomousInit)
                {
                    Logger.println("[CrevoRobot]: Autonomous Init", Logger.LogLevel.DEBUG);
//                    mAutonExecutor.setAuton(mAutonSelector.getSelected());
//                    mAutonExecutor.start();

                    mDisabledInit = false;
                    mAutonomousInit = true;
                }
            } else if (isOperatorControl())
            {
                if (!mTeleopInit)
                {
                    Logger.println("[CrevoRobot]: Teleop Init", Logger.LogLevel.DEBUG);

                    mDisabledInit = false;
                    mTeleopInit = true;
                }
            } else if (isDisabled())
            {
                if (!mDisabledInit)
                {
                    Logger.println("[CrevoLib]: Disabled Init", Logger.LogLevel.DEBUG);
                    mAutonExecutor.stop();
                    mSubManager.stopAllSubsystems();

                    mAutonomousInit = false;
                    mTeleopInit = false;
                    mDisabledInit = true;
                }
            }

            periodic();

            try
            {
                Thread.sleep(mPeriod, 0);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
    }
}
