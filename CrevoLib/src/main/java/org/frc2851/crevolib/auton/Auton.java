package org.frc2851.crevolib.auton;

import edu.wpi.first.wpilibj.Timer;

public abstract class Auton {
    private String name = "NULL Auton";
    protected Auton(String name)
    {
        this.name = name;
    }

    public String getName()
    {
        return name;
    }

    protected boolean isAlive = false;

    protected abstract void routine() throws AutonEndedException;

    public void run()
    {
        isAlive = true;
        try {
            routine();
        } catch (AutonEndedException e) {
            System.out.println("Auton Ended Unexpectedly");
            return;
        }
        done();
        System.out.println("Auton Complete");
    }

    public void done() { }

    public void stop() { isAlive = false; }

    public boolean isAlive() throws AutonEndedException
    {
        if (!isAlive) throw new AutonEndedException();
        return isAlive;
    }

    public void runAction(Action action) throws AutonEndedException {
        isAlive();
        action.start();

        while (isAlive() && !action.isFinished())
        {
            action.update();
            try { Thread.sleep(0, 1); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }
        action.done();
    }

    public void runSynchronousAction(Action a, Action b) throws AutonEndedException
    {
        isAlive();
        a.start();
        b.start();

        while (isAlive() && !a.isFinished() && !b.isFinished())
        {
            if (!a.isFinished()) a.update();
            if (!b.isFinished()) b.update();
            try { Thread.sleep(0, 1); }
            catch (InterruptedException e) { e.printStackTrace(); }
        }

        a.done();
        b.done();
    }
}
