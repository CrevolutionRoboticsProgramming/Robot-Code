package org.frc2851.crevolib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.frc2851.crevolib.autonomous.Auton;
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

    private Logger _logger = Logger.getInstance();
    private Vector<Subsystem> _subs = new Vector<>();
    private Vector<Auton> _autons = new Vector<>();
    private SendableChooser<Auton> _autonSelector = new SendableChooser<>();
    private Auton _selectedAuton = null;

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

    /**
     * Adds an auton to the list of autons pushed to the SmartDashboard
     * @param auton Auton to be added
     */
    protected void addAuton(Auton auton) { _autons.add(auton); }

    @Override
    public final void robotInit()
    {
        for (Subsystem s : _subs)
        {
            s.start();
            s.setCommand(null);
        }

        if (!_autons.isEmpty())
            for (Auton a : _autons) _autonSelector.addObject(a.getName(), a);

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
        if (!_autons.isEmpty())
        {
            _selectedAuton = _autonSelector.getSelected();
            _selectedAuton.start();
        }
    }

    @Override
    public final void teleopInit()
    {
        for (Subsystem s : _subs) s.setCommand(s.getTeleopCommand());
    }

    @Override
    public final void disabledInit()
    {
        if (_selectedAuton != null)
            _selectedAuton.stopAuton();
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
