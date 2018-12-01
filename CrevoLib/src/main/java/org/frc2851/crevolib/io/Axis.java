package org.frc2851.crevolib.io;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Represents an axis on a XBox Controller
 */
public class Axis
{
    /**
     * The operation mode of the axis.
     *
     * <ul>
     *     <li>Raw: Direct input from controller</li>
     *     <li>Inverted: Raw input * -1</li>
     *     <li>Shaped: Returns the raw value after it is passed through a shaper function</li>
     * </ul>
     */
    public enum AxisMode
    {
        RAW, INVERTED, SHAPED
    }

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
    private final AxisMode _mode;

    /**
     * Creates an axis (analog joystick input)
     * @param channel The channel the joystick is on (defined by the Driver Station)
     * @param id The ID of the axis
     * @param mode The behavior of the axis
     */
    public Axis(int channel, AxisID id, AxisMode mode)
    {
        _joy = new Joystick(channel);
        _id = id;
        _mode = mode;
    }

    /**
     * Returns the adjusted value of the axis. If the shaper value is {@code null}, the raw input is returned.
     * @param shaper Shaper Function (such as return val^2).
     * @return The adjusted axis value
     */
    public double get(InputShaper shaper)
    {
        // TODO: Remove shaped as a mode.
        double val = _joy.getRawAxis(_id.getID());
        switch (_mode)
        {
            case RAW:
            {
                return val;
            }

            case INVERTED:
            {
                return -val;
            }

            case SHAPED:
            {
                if (shaper == null) return val;
                return shaper.shape(val);
            }

            default:
                return 0;
        }
    }
}
