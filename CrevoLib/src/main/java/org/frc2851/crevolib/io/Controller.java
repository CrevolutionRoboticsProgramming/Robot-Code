package org.frc2851.crevolib.io;

import java.util.HashMap;

/**
 * Represents a XBox controller
 */
public class Controller
{
    private HashMap<Button.ButtonID, Button> _buttons = new HashMap<>();
    private HashMap<Axis.AxisID, Axis> _axis = new HashMap<>();
    private final int channel;

    /**
     * Creates a controller on the provided Driver Station channel
     * @param channel Channel id
     */
    public Controller(int channel)
    {
        this.channel = channel;
    }

    /**
     * Returns the state of the given button
     * @param id The ButtonID
     * @return The state of the given button
     */
    public boolean get(Button.ButtonID id)
    {
        if (!_buttons.containsKey(id))
        {
            System.out.println("Button[" + id.name() + "] is not configured");
            return false;
        }
        return _buttons.get(id).get();
    }

    /**
     * Returns the value of the given axis
     * @param id The AxisID
     * @param shaper The shaper function (should be given as a lambda expression)
     * @return The value of the given axis
     */
    @Deprecated
    public double get(Axis.AxisID id, InputShaper shaper)
    {
        if (!_axis.containsKey(id))
        {
            System.out.println("Axis[" + id.name() + "] is not configured");
            return 0;
        }
        return _axis.get(id).get(shaper);
    }

    /**
     * Returns the value of the given axis
     * @param id The AxisID
     * @return The value of the given axis
     */
    public double get(Axis.AxisID id)
    {
        if (!_axis.containsKey(id))
        {
            System.out.println("Axis[" + id.name() + "] is not configured");
            return 0;
        }
        return _axis.get(id).get();
    }

    /**
     * Configures the given button with the given mode
     * @param id The ButtonID
     * @param mode The ButtonMode
     */
    public void config(Button.ButtonID id, Button.ButtonMode mode)
    {
        if (!_buttons.containsKey(id)) _buttons.put(id, new Button(channel, id, mode));
    }

    /**
     * Configures the given button with the given mode
     * @param id The AxisID
     */
    public void config(Axis.AxisID id)
    {
        if (!_axis.containsKey(id)) _axis.put(id, new Axis(channel, id));
    }

    /**
     * Configures the given button with the given mode
     * @param id The AxisID
     * @param shaper The shaper function
     */
    public void config(Axis.AxisID id, InputShaper shaper)
    {
        if (!_axis.containsKey(id)) _axis.put(id, new Axis(channel, id, shaper));
    }

    /**
     * Sets the shaper function for the given axis
     * @param id The AxisID
     * @param shaper The shaper function
     */
    public void setShaper(Axis.AxisID id, InputShaper shaper)
    {
        if (!_axis.containsKey(id))
        {
            System.out.println("Axis[" + id.name() + "] is not configured");
            return;
        }

        _axis.get(id).setShaper(shaper);
    }
}