package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;//Bad Log,bad you are third party so that is very bad.
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

public class Hatcher extends Subsystem {

    private Button.ButtonID hatcherExtend = Button.ButtonID.RIGHT_BUMPER;
    private Button.ButtonID hatcherActuate = Button.ButtonID.LEFT_BUMPER;
    //I got rid of the extra buttons so that one extends and one actuates
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

        mController.config(hatcherExtend, Button.ButtonMode.TOGGLE);
        mController.config(hatcherActuate, Button.ButtonMode.TOGGLE);
        // I changed it from RAW to TOGGLE
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
                reset(); //Jason isn't sure that this is necessary "Worth the lost cycles of uncertainty"

                BadLog.createTopic("HatcherActuated", BadLog.UNITLESS, () -> mActuateSol.get() == DoubleSolenoid.Value.kReverse ? 1.0 : 0.0, "hide", "join:Hatcher Actuated ");
                BadLog.createTopic("HatcherExtended", BadLog.UNITLESS, () -> mExtendSol.get() == DoubleSolenoid.Value.kForward ? 1.0 : 0.0, "hide", "join:Hatcher Extended  ");
                //I added Badlog.
                return true;
            }

            @Override
            public void update() {
                if (mController.get(hatcherExtend)) {
                    mExtendSol.set(DoubleSolenoid.Value.kForward);
                } else {
                    mExtendSol.set(DoubleSolenoid.Value.kReverse);
                }

                if (mController.get(hatcherActuate)) {
                    mActuateSol.set(DoubleSolenoid.Value.kReverse);
                } else {
                    mActuateSol.set(DoubleSolenoid.Value.kForward);
                }
                // I got rid of the else if statements that were no longer needed since it was changed from RAW to TOGGLE.
            }

            @Override
            public void stop() {
                reset();
            }

        };
    }

}
