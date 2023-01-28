package org.frc2851.crevolib.io;

import edu.wpi.first.wpilibj.Joystick;

import java.util.function.DoubleFunction;

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

        AxisID(int id)
        {
            this.id = id;
        }

        /**
         * Returns the integer that corresponds with the getRawAxis() function
         *
         * @return id
         */
        public int getID()
        {
            return id;
        }
    }

    private Joystick mJoystick;
    private final AxisID mId;
    private DoubleFunction<Double> mShaper, mDeadbandShaper;

    /**
     * Creates an axis (analog joystick input)
     *
     * @param channel The channel the joystick is on (defined by the Driver Station)
     * @param id      The ID of the axis
     */
    Axis(int channel, AxisID id)
    {
        mJoystick = new Joystick(channel);
        mId = id;
        mShaper = (x) -> x;
        mDeadbandShaper = (x) -> x;
    }

    Axis(int channel, AxisID id, DoubleFunction<Double> shaper)
    {
        mJoystick = new Joystick(channel);
        mId = id;
        mShaper = shaper;
        mDeadbandShaper = (x) -> x;
    }

    void setShaper(DoubleFunction<Double> shaper)
    {
        mShaper = shaper;
    }

    void setDeadband(double deadband, boolean rescale)
    {
        if (rescale) mDeadbandShaper = x -> (x / Math.abs(x)) * ((Math.abs(x) - deadband) / (1 - deadband));
        else mDeadbandShaper = x -> (Math.abs(x) < deadband) ? 0 : x;
    }

    /**
     * Returns the adjusted value of the axis. If the shaper value is {@code null}, the raw input is returned.
     *
     * @param shaper Shaper Function (such as return val^2).
     * @return The adjusted axis value
     */
    @Deprecated
    double get(DoubleFunction<Double> shaper)
    {
        return shaper.apply(mJoystick.getRawAxis(mId.getID()));
    }

    double get()
    {
        return mShaper.apply(
                mDeadbandShaper.apply(mJoystick.getRawAxis(mId.getID()))
        );
    }
}
