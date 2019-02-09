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
 shows which button goes to which solenoid
 declares Solenoids and other variables
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
    /**
     reset the solenoids to off
     */
    private void reset() {
        mExtendSol.set(DoubleSolenoid.Value.kOff);
        mActuateSol.set(DoubleSolenoid.Value.kOff);
    }
    /**
     initializing that extend sol extends the arm and actuates grabs the hatch
     setting Buttons to toggle
     */
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
            /**
             Initialize badlog
              */
            @Override
            public boolean init() {
                reset();
                BadLog.createTopic("hatcher/actuate", BadLog.UNITLESS, () -> mActuateSol.get() == DoubleSolenoid.Value.kReverse ? 1.0 : 0.0, "hide", "join:hatcher/actuate Outputs");
                BadLog.createTopic("hatcher/extend", BadLog.UNITLESS, () -> mExtendSol.get() == DoubleSolenoid.Value.kForward ? 1.0 : 0.0, "hide", "join:hatcher/extend Outputs");

                return true;
            }
            /**
             connects the solenoid functions to there buttons on the controller
              */
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
