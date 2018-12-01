package org.frc2851.crevolib.autonomous;

//  TODO: Determine if the auton will consistently kill stuck action on shutdown

/**
 * An auton is a set of instructions that control the robot in autonomous mode. This class contains the code that runs
 * the auton. It is also the base class that all autons need to extend.
 */
public abstract class Auton extends Thread
{
    private Thread _thread;
    private boolean _autonActive = false;
    private String _name;

    protected Auton(String name) { _name = name; }

    protected abstract void init();
    protected abstract void routine() throws AutonEndedException;

    /**
     * Forces the auton to stop
     */
    public synchronized void stopAuton()
    {
        if (_autonActive)
        {
            _autonActive = false;
            _thread.stop();
        }
    }

    /**
     * Runs an action
     * @param action The action to run
     * @throws AutonEndedException Thrown if the autonomous period ended
     */
    protected void runAction(Action action) throws AutonEndedException
    {
        if (!_autonActive) throw new AutonEndedException();
        System.out.println("Auton[" + _name + "]: " + action.getName());
        action.init();
        while (!action.isFinished() && _autonActive) action.update();
    }

    @Override
    public void run()
    {
        init();
        try {
            routine();
        } catch (AutonEndedException ignore) { }
        System.out.println("Auton Ended");
    }

    /**
     * Starts the auton
     */
    @Override
    public void start()
    {
        _autonActive = true;
        if (_thread == null) _thread = new Thread(this, _name);
        _thread.start();
    }
}

/**
 * Thrown when the autonomous mode finished
 */
class AutonEndedException extends Exception {}
