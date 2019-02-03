package org.frc2851.crevolib.subsystem;

import edu.wpi.first.wpilibj.Notifier;
import org.frc2851.crevolib.Logger;

import java.util.Vector;

public class SubsystemManager
{
    private Vector<Subsystem> _subsystems = new Vector<>();
    private Notifier _notifier;
    private static SubsystemManager instance = new SubsystemManager();

    private SubsystemManager() {}
    public static SubsystemManager getInstance() { return instance; }

    public void addSubsystem(Subsystem s)
    {
        if (s == null) {
            Logger.println("SubsystemManager: Subsystem is null", Logger.LogLevel.ERROR);
            return;
        }
        _subsystems.add(s);
    }

    public synchronized void setTeleop() { for (Subsystem s : _subsystems) s.setCommand(s.getDefaultCommand()); }
    public synchronized void setDisabled() { for (Subsystem s : _subsystems) s.setCommand(null); }

    private synchronized void run()
    {
        for (Subsystem s : _subsystems) s.runCommand();
    }


    public void start()
    {
        _notifier = new Notifier(this::run);
        _notifier.startPeriodic(0.005);
    }
}
