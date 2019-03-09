package org.frc2851.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;

/**
 * Represents the hatcher subsystem
 * <p>
 * Controls:
 * A - Extension Toggle
 * B - Beak Toggle
 */
public class Hatcher extends Subsystem
{
    private enum ActuationState
    {
        OPEN(DoubleSolenoid.Value.kForward), CLOSED(DoubleSolenoid.Value.kReverse);

        final DoubleSolenoid.Value val;

        ActuationState(DoubleSolenoid.Value val)
        {
            this.val = val;
        }
    }

    private enum ExtensionState
    {
        STOWED(DoubleSolenoid.Value.kForward), EXTENDED(DoubleSolenoid.Value.kReverse);

        final DoubleSolenoid.Value val;

        ExtensionState(DoubleSolenoid.Value val)
        {
            this.val = val;
        }
    }

    private DoubleSolenoid mExtendSol, mActuateSol;
    private Controller mController = Constants.operator;
    private Constants mConstants = Constants.getInstance();

    private static Hatcher mInstance;

    /**
     * Initializes the Hatcher class with the name "Hatcher"
     */
    private Hatcher()
    {
        super("Hatcher");
    }

    /**
     * Returns the sole instance of the Hatcher class
     *
     * @return The instance of the Hatcher class
     */
    public static Hatcher getInstance()
    {
        if (mInstance == null) mInstance = new Hatcher();
        return mInstance;
    }

    /**
     * Initializes the controller, solenoids, and logging
     *
     * @return A boolean representing whether initialization has succeeded
     */
    @Override
    public boolean init()
    {
        mExtendSol = new DoubleSolenoid(mConstants.pcm, mConstants.ht_extendForward, mConstants.ht_extendReverse);
        mActuateSol = new DoubleSolenoid(mConstants.pcm, mConstants.ht_actuateForward, mConstants.ht_actuateReverse);

        mController.config(mConstants.ht_extend, Button.ButtonMode.TOGGLE);
        mController.config(mConstants.ht_actuate, Button.ButtonMode.TOGGLE);
        return true;
    }

    /**
     * Returns a command representing user control over the hatcher
     *
     * @return A command representing user control over the hatcher
     */
    @Override
    public Command getDefaultCommand()
    {
        return new Command()
        {
            ActuationState lastActuationState = ActuationState.CLOSED;
            ExtensionState lastExtensionState = ExtensionState.STOWED;

            @Override
            public String getName()
            {
                return "Teleop";
            }

            @Override
            public boolean isFinished()
            {
                return false;
            }

            @Override
            public boolean init()
            {
                return true;
            }

            @Override
            public void update()
            {
                ExtensionState extensionState = mController.get(mConstants.ht_extend) ? ExtensionState.EXTENDED : ExtensionState.STOWED;
                ActuationState actuationState = mController.get(mConstants.ht_actuate) ? ActuationState.OPEN : ActuationState.CLOSED;

                if (extensionState != lastExtensionState)
                    log("Updated extension state: " + extensionState.name(), Logger.LogLevel.DEBUG);
                if (actuationState != lastActuationState)
                    log("Updated actuation state: " + actuationState.name(), Logger.LogLevel.DEBUG);

                lastActuationState = actuationState;
                lastExtensionState = extensionState;

                mExtendSol.set(extensionState.val);
                mActuateSol.set(actuationState.val);
            }

            @Override
            public void stop()
            {

            }

        };
    }

}
