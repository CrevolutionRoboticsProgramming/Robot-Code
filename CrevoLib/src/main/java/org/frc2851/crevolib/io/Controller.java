package org.frc2851.crevolib.io;

import java.util.HashMap;

/**
 * Represents a XBox controller
 */
public class Controller
{
    private HashMap<Button.ButtonID, Button> mButtons = new HashMap<>();
    private HashMap<Axis.AxisID, Axis> mAxis = new HashMap<>();
    private final int mChannel;

    /**
     * Creates a controller on the provided Driver Station mChannel
     *
     * @param channel Channel id
     */
    public Controller(int channel)
    {
        this.mChannel = channel;
    }

    /**
     * Returns the state of the given button
     *
     * @param id The ButtonID
     * @return The state of the given button
     */
    public boolean get(Button.ButtonID id)
    {
        if (!mButtons.containsKey(id))
        {
            System.out.println("Button[" + id.name() + "] is not configured");
            return false;
        }
        return mButtons.get(id).get();
    }

    /**
     * Returns the value of the given axis
     *
     * @param id     The AxisID
     * @param shaper The shaper function (should be given as a lambda expression)
     * @return The value of the given axis
     */
    @Deprecated
    public double get(Axis.AxisID id, InputShaper shaper)
    {
        if (!mAxis.containsKey(id))
        {
            System.out.println("Axis[" + id.name() + "] is not configured");
            return 0;
        }
        return mAxis.get(id).get(shaper);
    }

    /**
     * Returns the value of the given axis
     *
     * @param id The AxisID
     * @return The value of the given axis
     */
    public double get(Axis.AxisID id)
    {
        if (!mAxis.containsKey(id))
        {
            System.out.println("Axis[" + id.name() + "] is not configured");
            return 0;
        }
        return mAxis.get(id).get();
    }

    /**
     * Configures the given button with the given mode
     *
     * @param id   The ButtonID
     * @param mode The ButtonMode
     */
    public void config(Button.ButtonID id, Button.ButtonMode mode)
    {
        if (!mButtons.containsKey(id)) mButtons.put(id, new Button(mChannel, id, mode));
    }

    /**
     * Configures the given button with the given mode
     *
     * @param id The AxisID
     */
    public void config(Axis.AxisID id)
    {
        if (!mAxis.containsKey(id)) mAxis.put(id, new Axis(mChannel, id));
    }

    /**
     * Configures the given button with the given mode
     *
     * @param id     The AxisID
     * @param shaper The shaper function
     */
    public void config(Axis.AxisID id, InputShaper shaper)
    {
        if (!mAxis.containsKey(id)) mAxis.put(id, new Axis(mChannel, id, shaper));
    }

    /**
     * Sets the shaper function for the given axis
     *
     * @param id     The AxisID
     * @param shaper The shaper function
     */
    public void setShaper(Axis.AxisID id, InputShaper shaper)
    {
        if (!mAxis.containsKey(id))
        {
            System.out.println("Axis[" + id.name() + "] is not configured");
            return;
        }

        mAxis.get(id).setShaper(shaper);
    }

    public void setDeadband(Axis.AxisID id, double deadband, boolean rescale)
    {
        if (!mAxis.containsKey(id))
        {
            System.out.println("Axis[" + id.name() + "] is not configured");
            return;
        }

        mAxis.get(id).setDeadband(deadband, rescale);
    }

    public void setToggleState(Button.ButtonID id, boolean state)
    {
        if (!mAxis.containsKey(id))
        {
            System.out.println("Axis[" + id.name() + "] is not configured");
            return;
        }

        mButtons.get(id).setToggleState(state);
    }
}