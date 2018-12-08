package org.frc2851.crevolib.subsystem;

import edu.wpi.first.wpilibj.Timer;
import org.frc2851.crevolib.Logger;

import java.util.Vector;

public class SubsystemManager extends Thread
{
    private Vector<Subsystem> _subsystems = new Vector<>();

    private Thread _thread;

    public SubsystemManager()
    {

    }

    public void addSubsystem(Subsystem s) { _subsystems.add(s); }

    public void setTeleop() { for (Subsystem s : _subsystems) s.setCommand(s.getTeleopCommand()); }
    public void setDisabled() { for (Subsystem s : _subsystems) s.setCommand(null); }

    @Override
    public synchronized void run()
    {
        while (true)
        {
            double startTime = Timer.getFPGATimestamp();

            for (Subsystem s : _subsystems) s.runCommand();

            double dt = Timer.getFPGATimestamp() - startTime;
            try {
                if (dt < 0.005) Thread.sleep(5 - (int)(dt * 1000));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void start()
    {
        if (_thread == null)
        {
            Logger.println("|---- Subsystem Manager Starting ----|", Logger.LogLevel.DEBUG);

            for (Subsystem s : _subsystems) s.init();

            _thread = new Thread(this, "Subsystem Manager");
            _thread.start();
        }
    }
}
