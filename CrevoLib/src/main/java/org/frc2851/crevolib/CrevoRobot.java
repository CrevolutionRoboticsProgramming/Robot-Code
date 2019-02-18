package org.frc2851.crevolib;

import badlog.lib.BadLog;
import badlog.lib.DataInferMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.frc2851.crevolib.auton.Auton;
import org.frc2851.crevolib.auton.AutonExecutor;
import org.frc2851.crevolib.motion.BadMotionProfileException;
import org.frc2851.crevolib.motion.MotionProfile;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.crevolib.subsystem.SubsystemManager;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

public class CrevoRobot extends TimedRobot
{
    private static final String MOTION_PROFILE_DIR = "/home/lvuser/motion/";

    private AutonExecutor _executor = new AutonExecutor();
    private SendableChooser<Auton> _autonSelector = new SendableChooser<>();
    private SubsystemManager _subManager = SubsystemManager.getInstance();

    private static HashMap<String, MotionProfile> _motionProfiles = new HashMap<>();

    private BadLog badLog;
    private long startTimeNs;
    private long lastLog, currentTimeMillis;

    private static boolean mIsEnabled = false;


    /**
     * Adds a subsystem to the robots routine. Also adds it to the logger.
     * @param subsystem Subsystem to be added
     */
    protected void addSubsystem(Subsystem subsystem)
    {
        Logger.println("Registered Subsystem: " + subsystem.toString(), Logger.LogLevel.DEBUG);
        _subManager.addSubsystem(subsystem);
    }

    /**
     * Adds an autonomous phase into the robot's routine. It will also add it into the logger
     * @param auton
     */
    protected void addAuton(Auton auton)
    {
        Logger.println("Registered Auton: " + auton.getName(), Logger.LogLevel.DEBUG);
        _autonSelector.addObject(auton.getName(), auton);
    }

    protected CrevoRobot()
    {
        badLog = BadLog.init("/home/lvuser/log.bag");
    }

    @Override
    public final void robotInit()
    {
        startTimeNs = System.nanoTime();
        lastLog = System.currentTimeMillis();

        Logger.start();
        Logger.println("Robot Init", Logger.LogLevel.DEBUG);
        BadLog.createTopic("Match Time", "s", () -> DriverStation.getInstance().getMatchTime());
        BadLog.createTopicSubscriber("Time", "s", DataInferMode.DEFAULT, "hide", "delta", "xaxis");

        ArrayList<File> files = FileUtil.getFiles(MOTION_PROFILE_DIR, true);
        for (File f : files)
        {
            String name = f.getName().split("\\.")[0];
            try {
                _motionProfiles.put(name, new MotionProfile(f));
            } catch (BadMotionProfileException ignored) { }
        }

        _subManager.start();

        badLog.finishInitialization();
    }

    @Override
    public final void autonomousInit()
    {
        Logger.println("Autonomous Init", Logger.LogLevel.DEBUG);
        _executor.setAuton(_autonSelector.getSelected());
        _executor.start();
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
        _executor.stop();
    }

    // TODO: Move all periodic tasks into the robot manager

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
     * @param name The full name of the file, excluding file extensions
     * @return The selected motion profile. If the motion profile specified does not exist,
     * the function returns {@code null}.
     */
    public static MotionProfile getMotionProfile(String name)
    {
        return _motionProfiles.getOrDefault(name, null);
    }

    private void periodic()
    {
        double time = ((double) (System.nanoTime() - startTimeNs) / 1000000000d);
        BadLog.publish("Time", time);

        badLog.updateTopics();

        currentTimeMillis = System.currentTimeMillis();
        if (!this.isDisabled() || (currentTimeMillis - lastLog >= 1000)) {
            lastLog = System.currentTimeMillis();
            badLog.log();
        }

        mIsEnabled = isEnabled();
    }

    public static boolean isRunning() {
        return mIsEnabled;
    }
}
