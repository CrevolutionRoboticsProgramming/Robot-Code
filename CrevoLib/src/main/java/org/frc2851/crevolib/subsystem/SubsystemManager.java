package org.frc2851.crevolib.subsystem;

import edu.wpi.first.wpilibj.Notifier;
import org.frc2851.crevolib.Logger;

import java.util.Vector;

public class SubsystemManager
{
    private Vector<Subsystem> _subsystems = new Vector<>();
    private Notifier _notifier;
    private static SubsystemManager mInstance;

    private SubsystemManager()
    {
    }

    public static SubsystemManager getInstance()
    {
        if (mInstance == null) mInstance = new SubsystemManager();
        return mInstance;
    }

    public void addSubsystem(Subsystem s)
    {
        if (s == null)
        {
            Logger.println("[SubsystemManager]: Subsystem is null", Logger.LogLevel.ERROR);
            return;
        }
        Logger.println("[SubsystemManager]: Added " + s.getName(), Logger.LogLevel.DEBUG);
        _subsystems.add(s);
    }

    private synchronized void run()
    {
        for (Subsystem s : _subsystems) s.runCommand();
    }

    public void start()
    {
        for (Subsystem s : _subsystems)
        {
            if (!s.init())
                Logger.println("[SubsystemManger]: Could not initialize " + s.getName(), Logger.LogLevel.ERROR);
            else Logger.println("[SubsystemManager]: Successfully initialized " + s.getName(), Logger.LogLevel.DEBUG);
        }

        _notifier = new Notifier(this::run);
        _notifier.startPeriodic(0.005);
    }

    /**
     * Stops auxiliary commands of all subsystems. Default commands continue to run.
     */
    public void stopAllSubsystems()
    {
        for (Subsystem s : _subsystems) s.stopAuxiliaryCommand();
    }
}
