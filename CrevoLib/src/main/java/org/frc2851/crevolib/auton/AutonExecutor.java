package org.frc2851.crevolib.auton;

import org.frc2851.crevolib.Logger;

/**
 * Executes the autonomous period.
 */
public class AutonExecutor
{
    private Auton mAuton;
    private Thread mThread = null;

    public void setAuton(Auton auton)
    {
        if (auton != null) Logger.println("Auton Set: " + auton.getName(), Logger.LogLevel.DEBUG);
        mAuton = auton;
    }

    public void start()
    {
        if (mThread == null)
        {
            mThread = new Thread(() -> {
                if (mAuton != null)
                    mAuton.run();
            });

            mThread.start();
        }
    }

    public void stop()
    {
        if (mAuton != null && mAuton.isAlive)
            mAuton.stop();

        mThread = null;
    }
}
