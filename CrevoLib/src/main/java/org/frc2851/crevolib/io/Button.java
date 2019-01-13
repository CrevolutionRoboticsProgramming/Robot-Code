<<<<<<< HEAD
package org.frc2851.crevolib.io;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Represents a button on a XBox controller
 */
public class Button
{
    /**
     * The operation mode of the button.
     *
     * <ul>
     *     <li>Toggle: Toggles state on press</li>
     *     <li>On Press: Returns true when first pressed</li>
     *     <li>On Release: Returns true when first released</li>
     *     <li>Raw: Returns the raw state of the button</li>
     * </ul>
     */
    public enum ButtonMode
    {
        TOGGLE, ON_PRESS, ON_RELEASE, RAW
    }

    /**
     * The name of the button
     */
    public enum ButtonID
    {
        A(1), B(2), X(3), Y(4), START(8), SELECT(7), LEFT_BUMPER(5), RIGHT_BUMPER(6),
        LEFT_JOYSTICK(9), RIGHT_JOYSTICK(10);

        private int id;
        ButtonID(int id) { this.id = id; }

        /**
         * Returns the integer that corresponds with the getRawButton() function
         * @return id
         */
        public int getID() { return id; }
    }

    private Joystick _joy;
    private ButtonID _id;
    private ButtonMode _mode;
    private boolean _isToggled = false, _lastState = false;

    /**
     * Creates an button (digital joystick input)
     * @param channel The channel the joystick is on (defined by the Driver Station)
     * @param id The ID of the button
     * @param mode The behavior of the button
     */
    Button(int channel, ButtonID id, ButtonMode mode)
    {
        _joy = new Joystick(channel);
        _id = id;
        _mode = mode;
    }

    /**
     * Returns the adjusted state of the button
     * @return button state
     */
    boolean get()
    {
        boolean state = _joy.getRawButton(_id.getID());
        switch (_mode)
        {
            case RAW:
            {
                return state;
            }

            case TOGGLE:
            {
                if (state && !_lastState) _isToggled = !_isToggled;
                _lastState = state;
                return _isToggled;
            }

            case ON_PRESS:
            {
                boolean isPressed = false;
                if (state && !_lastState) isPressed = true;
                _lastState = state;
                return isPressed;
            }

            case ON_RELEASE:
            {
                boolean isReleased = false;
                if (!state && _lastState) isReleased = true;
                _lastState = state;
                return isReleased;
            }

            default:
                return false;
        }
    }
}
=======
package org.frc2851.crevolib.io;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Represents a button on a XBox controller
 */
public class Button
{
    /**
     * The operation mode of the button.
     *
     * <ul>
     *     <li>Toggle: Toggles state on press</li>
     *     <li>On Press: Returns true when first pressed</li>
     *     <li>On Release: Returns true when first released</li>
     *     <li>Raw: Returns the raw state of the button</li>
     * </ul>
     */
    public enum ButtonMode
    {
        TOGGLE, ON_PRESS, ON_RELEASE, RAW
    }

    /**
     * The name of the button
     */
    public enum ButtonID
    {
        A(1), B(2), X(3), Y(4), START(8), SELECT(7), LEFT_BUMPER(5), RIGHT_BUMPER(6),
        LEFT_JOYSTICK(9), RIGHT_JOYSTICK(10);

        private int id;
        ButtonID(int id) { this.id = id; }

        /**
         * Returns the integer that corresponds with the getRawButton() function
         * @return id
         */
        public int getID() { return id; }
    }

    private Joystick _joy;
    private ButtonID _id;
    private ButtonMode _mode;
    private boolean _isToggled = false, _lastState = false;

    /**
     * Creates an button (digital joystick input)
     * @param channel The channel the joystick is on (defined by the Driver Station)
     * @param id The ID of the button
     * @param mode The behavior of the button
     */
    Button(int channel, ButtonID id, ButtonMode mode)
    {
        _joy = new Joystick(channel);
        _id = id;
        _mode = mode;
    }

    /**
     * Returns the adjusted state of the button
     * @return button state
     */
    boolean get()
    {
        boolean state = _joy.getRawButton(_id.getID());
        switch (_mode)
        {
            case RAW:
            {
                return state;
            }

            case TOGGLE:
            {
                if (state && !_lastState) _isToggled = !_isToggled;
                _lastState = state;
                return _isToggled;
            }

            case ON_PRESS:
            {
                boolean isPressed = false;
                if (state && !_lastState) isPressed = true;
                _lastState = state;
                return isPressed;
            }

            case ON_RELEASE:
            {
                boolean isReleased = false;
                if (!state && _lastState) isReleased = true;
                _lastState = state;
                return isReleased;
            }

            default:
                return false;
        }
    }
}
>>>>>>> 291f823785993974c90dbca1d834dba43d07ee46
