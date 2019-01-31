package org.frc2851.robot.subsystems;

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

    private Button.ButtonID extendOutButton = Button.ButtonID.X;
    private Button.ButtonID extendInButton = Button.ButtonID.A;
    private Button.ButtonID actuateOutButton = Button.ButtonID.Y;
    private Button.ButtonID actuateInButton = Button.ButtonID.B;

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

        mController.config(extendOutButton, Button.ButtonMode.RAW);
        mController.config(extendInButton, Button.ButtonMode.RAW);
        mController.config(actuateOutButton, Button.ButtonMode.RAW);
        mController.config(actuateInButton, Button.ButtonMode.RAW);

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
                return true;
            }

            @Override
            public void update() {
                if (mController.get(extendOutButton)) {
                    mExtendSol.set(DoubleSolenoid.Value.kForward);
                } else if (mController.get(extendInButton)) {
                    mExtendSol.set(DoubleSolenoid.Value.kReverse);
                } else {
                    mExtendSol.set(DoubleSolenoid.Value.kOff);
                }

                if (mController.get(actuateOutButton)) {
                    mActuateSol.set(DoubleSolenoid.Value.kReverse);
                } else if (mController.get(actuateInButton)) {
                    mActuateSol.set(DoubleSolenoid.Value.kForward);
                } else {
                    mActuateSol.set(DoubleSolenoid.Value.kOff);
                }
            }

            @Override
            public void stop() {
                reset();
            }

        };
    }

}
