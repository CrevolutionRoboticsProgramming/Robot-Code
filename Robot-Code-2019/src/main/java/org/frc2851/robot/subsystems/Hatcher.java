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
 * TODO: Make buttons toggled instead of raw
 * Moving the mechanism forward should be controlled by one button and
 * actuation of the end should also be controlled by one. Driving the
 * solenoid backwards will close it, driving it forward will open it.
 * Change the binds in Constants, too
 */

public class Hatcher extends Subsystem {

    private Button.ButtonID extendButton = Button.ButtonID.RIGHT_BUMPER;
    private Button.ButtonID actuateButton = Button.ButtonID.LEFT_BUMPER;

    private DoubleSolenoid mExtendSol, mActuateSol;
    private Controller mController = Robot.operator;
    private Constants mConstants = Constants.getInstance();

    private static Hatcher _instance = new Hatcher();

    private Hatcher() {
        super("Hatcher");
    }

    public static Hatcher getInstance() {
        return _instance;
    }

    private void reset() {
        mExtendSol.set(DoubleSolenoid.Value.kOff);
        mActuateSol.set(DoubleSolenoid.Value.kOff);
    }

    @Override
    public boolean init() {
        mExtendSol = new DoubleSolenoid(mConstants.pcmID, mConstants.extendHatcherForward, mConstants.extendHatcherReverse);
        mActuateSol = new DoubleSolenoid(mConstants.pcmID, mConstants.actuateHatcherForward, mConstants.actuateHatcherReverse);

        reset();

        mController.config(extendButton, Button.ButtonMode.TOGGLE);
        mController.config(actuateButton, Button.ButtonMode.TOGGLE);

        return true;
    }

    @Override
    public Command getTeleopCommand() {
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
                BadLog.createTopic("hatcher/actuate", BadLog.UNITLESS, () -> mActuateSol.get() == DoubleSolenoid.Value.kReverse ? 1.0 : 0.0, "hide", "join:hatcher/actuate Outputs");
                BadLog.createTopic("hatcher/extend", BadLog.UNITLESS, () -> mExtendSol.get() == DoubleSolenoid.Value.kForward ? 1.0 : 0.0, "hide", "join:hatcher/extend Outputs");

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
