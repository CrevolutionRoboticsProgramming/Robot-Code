package org.frc2851.crevolib;

import edu.wpi.first.wpilibj.IterativeRobot;
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

public class CrevoRobot extends IterativeRobot
{
    private static final String MOTION_PROFILE_DIR = "/home/lvuser/motion/";

    private AutonExecutor _executor = new AutonExecutor();
    private SendableChooser<Auton> _autonSelector = new SendableChooser<>();
    private SubsystemManager _subManager = new SubsystemManager();

    private static HashMap<String, MotionProfile> _motionProfiles = new HashMap<>();

    /**
     * Adds a subsystem to the robots routine. Also adds it to the logger.
     * @param subsystem Subsystem to be added
     */
    protected void addSubsystem(Subsystem subsystem)
    {
        Logger.println("Registered Subsystem: " + subsystem.toString(), Logger.LogLevel.DEBUG);
        _subManager.addSubsystem(subsystem);
    }

    protected void addAuton(Auton auton)
    {
        Logger.println("Registered Auton: " + auton.getName(), Logger.LogLevel.DEBUG);
        _autonSelector.addObject(auton.getName(), auton);
    }

    @Override
    public final void robotInit()
    {
        Logger.start();
        Logger.println("Robot Init", Logger.LogLevel.DEBUG);

        _subManager.start();

        ArrayList<File> files = FileUtil.getFiles(MOTION_PROFILE_DIR, true);
        for (File f : files)
        {
            String name = f.getName().split("\\.")[0];
            try {
                _motionProfiles.put(name, new MotionProfile(f));
                Logger.println("MotionProfile: " + name, Logger.LogLevel.DEBUG);
            } catch (BadMotionProfileException ignored) { }
        }
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
        _subManager.setTeleop();
    }

    @Override
    public final void disabledInit()
    {
        Logger.println("Disabled Init", Logger.LogLevel.DEBUG);
        _executor.stop();
        _subManager.setDisabled();
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
}
