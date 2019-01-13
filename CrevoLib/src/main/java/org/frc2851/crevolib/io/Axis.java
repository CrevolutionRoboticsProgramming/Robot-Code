<<<<<<< HEAD
package org.frc2851.crevolib.io;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Represents an axis on a XBox Controller
 */
public class Axis
{
    /**
     * The name of the axis
     */
    public enum AxisID
    {
        LEFT_X(0), LEFT_Y(1), RIGHT_X(4), RIGHT_Y(5), LEFT_TRIGGER(2), RIGHT_TRIGGER(3);

        private int id;
        AxisID(int id) { this.id = id; }

        /**
         * Returns the integer that corresponds with the getRawAxis() function
         * @return id
         */
        public int getID() { return id; }
    }

    private Joystick _joy;
    private final AxisID _id;

    /**
     * Creates an axis (analog joystick input)
     * @param channel The channel the joystick is on (defined by the Driver Station)
     * @param id The ID of the axis
     */
    Axis(int channel, AxisID id)
    {
        _joy = new Joystick(channel);
        _id = id;
    }

    /**
     * Returns the adjusted value of the axis. If the shaper value is {@code null}, the raw input is returned.
     * @param shaper Shaper Function (such as return val^2).
     * @return The adjusted axis value
     */
    double get(InputShaper shaper)
    {
        return shaper.shape(_joy.getRawAxis(_id.getID()));
    }

    double get()
    {
        return _joy.getRawAxis(_id.getID());
    }
}
=======
package org.frc2851.crevolib.io;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Represents an axis on a XBox Controller
 */
public class Axis
{
    /**
     * The name of the axis
     */
    public enum AxisID
    {
        LEFT_X(0), LEFT_Y(1), RIGHT_X(4), RIGHT_Y(5), LEFT_TRIGGER(2), RIGHT_TRIGGER(3);

        private int id;
        AxisID(int id) { this.id = id; }

        /**
         * Returns the integer that corresponds with the getRawAxis() function
         * @return id
         */
        public int getID() { return id; }
    }

    private Joystick _joy;
    private final AxisID _id;

    /**
     * Creates an axis (analog joystick input)
     * @param channel The channel the joystick is on (defined by the Driver Station)
     * @param id The ID of the axis
     */
    Axis(int channel, AxisID id)
    {
        _joy = new Joystick(channel);
        _id = id;
    }

    /**
     * Returns the adjusted value of the axis. If the shaper value is {@code null}, the raw input is returned.
     * @param shaper Shaper Function (such as return val^2).
     * @return The adjusted axis value
     */
    double get(InputShaper shaper)
    {
        return shaper.shape(_joy.getRawAxis(_id.getID()));
    }

    double get()
    {
        return _joy.getRawAxis(_id.getID());
    }
}
>>>>>>> 291f823785993974c90dbca1d834dba43d07ee46
