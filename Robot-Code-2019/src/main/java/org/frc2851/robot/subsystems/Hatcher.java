package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

public class Hatcher extends Subsystem {

    private Button.ButtonID hatcherExtend = Button.ButtonID.RIGHT_BUMPER;
    private Button.ButtonID hatcherActuate = Button.ButtonID.LEFT_BUMPER;

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

        mController.config(hatcherExtend, Button.ButtonMode.RAW);
        mController.config(hatcherActuate, Button.ButtonMode.TOGGLE);

        reset();

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

            @Override
            public boolean init() {
                reset();

            //    BadLog.createTopic("Hatcher Actuated", BadLog.UNITLESS, () -> mActuateSol.get() == DoubleSolenoid.Value.kReverse ? 1.0 : 0.0, "hide", "join:Hatcher Actuated ");
            //    BadLog.createTopic("Hatcher Extended", BadLog.UNITLESS, () -> mExtendSol.get() == DoubleSolenoid.Value.kForward ? 1.0 : 0.0, "hide", "join:Hatcher Extended  ");
                return true;
            }

            @Override
            public void update() {
                if (mController.get(hatcherExtend)) {
                    mExtendSol.set(DoubleSolenoid.Value.kForward);
                    log("Hatcher Actuated", Logger.LogLevel.DEBUG);
                } else {
                    mExtendSol.set(DoubleSolenoid.Value.kReverse);
                }

                if (mController.get(hatcherActuate)) {
                    mActuateSol.set(DoubleSolenoid.Value.kReverse);
                    log("Hatcher Extended", Logger.LogLevel.DEBUG);
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
