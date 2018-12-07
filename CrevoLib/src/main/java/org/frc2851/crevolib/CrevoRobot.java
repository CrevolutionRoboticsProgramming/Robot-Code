package org.frc2851.crevolib;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.frc2851.crevolib.auton.Auton;
import org.frc2851.crevolib.auton.AutonExecutor;
import org.frc2851.crevolib.logging.Logger;
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

    private Logger _logger = Logger.getInstance();
    private Vector<Subsystem> _subs = new Vector<>();

    private static HashMap<String, MotionProfile> _motionProfiles = new HashMap<>();


    /**
     * Adds a subsystem to the robots routine. Also adds it to the logger.
     * @param subsystem Subsystem to be added
     */
    protected void addSubsystem(Subsystem subsystem)
    {
        _subs.add(subsystem);
        _logger.addWritable(subsystem);
    }

    protected void addAuton(Auton auton) { _autonSelector.addObject(auton.getName(), auton); }

    @Override
    public final void robotInit()
    {
        for (Subsystem s : _subs)
        {
            s.start();
            s.setCommand(null);
        }


        ArrayList<File> files = FileUtil.getFiles(MOTION_PROFILE_DIR, true);
        for (File f : files)
        {
            String name = f.getName().split("\\.")[0];
            try { _motionProfiles.put(name, new MotionProfile(f)); }
            catch (BadMotionProfileException ignored) { }
        }

        _logger.start();
    }

    @Override
    public final void autonomousInit()
    {
        _executor.setAuton(_autonSelector.getSelected());
    }

    @Override
    public final void teleopInit()
    {
        for (Subsystem s : _subs) s.setCommand(s.getTeleopCommand());
    }

    @Override
    public final void disabledInit()
    {
        _executor.stop();
        for (Subsystem s :  _subs) s.setCommand(null);
    }

    /**
     * Returns a motion profile fetched from the motion profile directory.
     * @param name The full name of the file, excluding file extensions
     * @return The selected motion profile. If the motion profile specified does not exist,
     * the function returns {@code null}.
     */
    public static MotionProfile getMotionProfile(String name) {
        return _motionProfiles.getOrDefault(name, null);
    }
}
