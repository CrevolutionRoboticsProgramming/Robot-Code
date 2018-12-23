package org.frc2851.crevolib;

import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.frc2851.crevolib.auton.Auton;
import org.frc2851.crevolib.auton.AutonExecutor;
import org.frc2851.crevolib.motion.BadMotionProfileException;
import org.frc2851.crevolib.motion.MotionProfile;
import org.frc2851.crevolib.subsystem.Subsystem;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Vector;

public class CrevoRobot extends IterativeRobot
{
    private static final String MOTION_PROFILE_DIR = "/home/lvuser/motion/";

    private AutonExecutor _executor = new AutonExecutor();
    private SendableChooser<Auton> _autonSelector = new SendableChooser<>();
    private Vector<Subsystem> _subs = new Vector<>();

    private static HashMap<String, MotionProfile> _motionProfiles = new HashMap<>();

    protected BadLog badLog;

    /**
     * Adds a subsystem to the robots routine. Also adds it to the logger.
     * @param subsystem Subsystem to be added
     */
    protected void addSubsystem(Subsystem subsystem)
    {
        Logger.println("Registered Subsystem: " + subsystem.toString(), Logger.LogLevel.DEBUG);
        _subs.add(subsystem);
    }

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
        Logger.start();
        Logger.println("Robot Init", Logger.LogLevel.DEBUG);

        BadLog.createTopic("Match Time", "s", () -> DriverStation.getInstance().getMatchTime());

        for (Subsystem s : _subs)
        {
            s.start();
            s.setCommand(null);
        }

        ArrayList<File> files = FileUtil.getFiles(MOTION_PROFILE_DIR, true);
        for (File f : files)
        {
            String name = f.getName().split("\\.")[0];
            try {
                _motionProfiles.put(name, new MotionProfile(f));
            } catch (BadMotionProfileException ignored) { }
        }

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
    public final void teleopInit()
    {
        Logger.println("Teleop Init", Logger.LogLevel.DEBUG);
        for (Subsystem s : _subs) s.setCommand(s.getTeleopCommand());
    }

    @Override
    public final void disabledInit()
    {
        Logger.println("Disabled Init", Logger.LogLevel.DEBUG);
        _executor.stop();
        for (Subsystem s : _subs) s.setCommand(null);
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
        badLog.updateTopics();
        badLog.log();
    }
}
