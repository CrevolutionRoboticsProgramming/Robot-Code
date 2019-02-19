package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

/**
 * Represents the Hatcher subsystem
 */
public class Hatcher extends Subsystem
{

    private DoubleSolenoid mExtendSol, mActuateSol;
    private Controller mController = Robot.operator;
    private Constants mConstants = Constants.getInstance();

    private static Hatcher _instance = new Hatcher();

    private DoubleSolenoid.Value extendState = DoubleSolenoid.Value.kOff, lastExtendState = DoubleSolenoid.Value.kOff;
    private DoubleSolenoid.Value actuateState = DoubleSolenoid.Value.kOff, lastActuateState = DoubleSolenoid.Value.kOff;

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
        return _instance;
    }

    /**
     * Resets the solenoids
     */
    private void reset()
    {
        mExtendSol.set(DoubleSolenoid.Value.kOff);
        mActuateSol.set(DoubleSolenoid.Value.kOff);
    }

    /**
     * Initializes the controller, solenoids, and logging
     *
     * @return A boolean representing whether initialization has succeeded
     */
    @Override
    public boolean init()
    {
        mExtendSol = new DoubleSolenoid(mConstants.pcmID, mConstants.extendHatcherForward, mConstants.extendHatcherReverse);
        mActuateSol = new DoubleSolenoid(mConstants.pcmID, mConstants.actuateHatcherForward, mConstants.actuateHatcherReverse);

        mController.config(mConstants.hatcherExtendButton, Button.ButtonMode.RAW);
        mController.config(mConstants.hatcherActuateButton, Button.ButtonMode.TOGGLE);

        BadLog.createTopic("Hatcher Actuated", BadLog.UNITLESS, () -> mActuateSol.get() == DoubleSolenoid.Value.kReverse ? 1.0 : 0.0, "hide", "join:hatcher/actuate Outputs");
        BadLog.createTopic("Hatcher Extended", BadLog.UNITLESS, () -> mExtendSol.get() == DoubleSolenoid.Value.kForward ? 1.0 : 0.0, "hide", "join:hatcher/extend Outputs");


        reset();

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
                reset();
                return true;
            }

            @Override
            public void update()
            {
                if (mController.get(mConstants.hatcherExtendButton))
                {
                    extendState = DoubleSolenoid.Value.kForward;
                } else
                {
                    extendState = DoubleSolenoid.Value.kReverse;
                }

                if (mController.get(mConstants.hatcherActuateButton))
                {
                    actuateState = DoubleSolenoid.Value.kReverse;
                } else
                {
                    actuateState = DoubleSolenoid.Value.kForward;
                }

                mExtendSol.set(extendState);
                mActuateSol.set(actuateState);

                if (extendState != lastExtendState)
                {
                    log("Extend solenoid set to " + extendState.toString(), Logger.LogLevel.DEBUG);
                }
                if (actuateState != lastActuateState)
                {
                    log("Actuate solenoid set to " + actuateState.toString(), Logger.LogLevel.DEBUG);
                }

                lastExtendState = extendState;
                lastActuateState = actuateState;
            }

            @Override
            public void stop()
            {
                reset();
            }

        };
    }

}
