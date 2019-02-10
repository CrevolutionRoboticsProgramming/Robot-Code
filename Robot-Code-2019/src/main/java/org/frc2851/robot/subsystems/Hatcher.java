package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;


/**
 * Represents the Hatcher subsystem
 */
public class Hatcher extends Subsystem {
    private Button.ButtonID extendButton = Button.ButtonID.RIGHT_BUMPER;
    private Button.ButtonID actuateButton = Button.ButtonID.LEFT_BUMPER;

    private DoubleSolenoid mExtendSol, mActuateSol;
    private Controller mController = Robot.operator;
    private Constants mConstants = Constants.getInstance();

    private static Hatcher _instance = new Hatcher();

    /**
     * Initializes the Hatcher class with the name "Hatcher"
     */
    private Hatcher() {
        super("Hatcher");
    }

    /**
     * Returns the sole instance of the Hatcher class
     * @return The instance of the Hatcher class
     */
    public static Hatcher getInstance() {
        return _instance;
    }

    /**
     * Resets the solenoids
     */
    private void reset() {
        mExtendSol.set(DoubleSolenoid.Value.kOff);
        mActuateSol.set(DoubleSolenoid.Value.kOff);
    }

    /**
     * Initializes the controller, solenoids, and logging
     * @return A boolean representing whether initialization has succeeded
     */
    @Override
    public boolean init() {
        mExtendSol = new DoubleSolenoid(mConstants.pcmID, mConstants.extendHatcherForward, mConstants.extendHatcherReverse);
        mActuateSol = new DoubleSolenoid(mConstants.pcmID, mConstants.actuateHatcherForward, mConstants.actuateHatcherReverse);

        reset();

        mController.config(extendButton, Button.ButtonMode.TOGGLE);
        mController.config(actuateButton, Button.ButtonMode.TOGGLE);

        BadLog.createTopic("hatcher/actuate", BadLog.UNITLESS, () -> mActuateSol.get() == DoubleSolenoid.Value.kReverse ? 1.0 : 0.0, "hide", "join:hatcher/actuate Outputs");
        BadLog.createTopic("hatcher/extend", BadLog.UNITLESS, () -> mExtendSol.get() == DoubleSolenoid.Value.kForward ? 1.0 : 0.0, "hide", "join:hatcher/extend Outputs");

        return true;
    }

    /**
     * Returns a command representing user control over the hatcher
     * @return A command representing user control over the hatcher
     */
    @Override
    public Command getDefaultCommand() {
        return new Command() {

            @Override
            public String getName() {
                return "Teleop";
            }

            @Override
            public boolean isFinished() {
                return false;
            }

            @Override
            public boolean init() {
                reset();

                return true;
            }

            @Override
            public void update() {
                if (mController.get(extendButton)) {
                    mExtendSol.set(DoubleSolenoid.Value.kForward);
                } else {
                    mExtendSol.set(DoubleSolenoid.Value.kReverse);
                }

                if (mController.get(actuateButton)) {
                    mActuateSol.set(DoubleSolenoid.Value.kReverse);
                } else {
                    mActuateSol.set(DoubleSolenoid.Value.kForward);
                }
            }

            @Override
            public void stop() {
                reset();
            }

        };
    }

}
