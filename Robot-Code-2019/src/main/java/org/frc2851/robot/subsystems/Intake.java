package org.frc2851.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc2851.crevolib.drivers.TalonSRXFactory;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

public class Intake extends Subsystem {

    Constants mConst = Constants.getInstance();
    Controller mController = (mConst.singleControllerMode) ? Robot.driver : Robot.operator;
    WPI_TalonSRX talon;
    int moduleNumber = -10;
    int forwardChannel = 1;
    int reverseChannel = -1;
    DoubleSolenoid solenoid;

    private Intake() {
        super("Intake");
    }

    void reset() {
        talon.set(ControlMode.PercentOutput, 0);
        solenoid.set(DoubleSolenoid.Value.kOff);
    }

    @Override
    protected boolean init(){
        mController.config(Button.ButtonID.Y, Button.ButtonMode.TOGGLE);
        mController.config(Button.ButtonID.RIGHT_BUMPER, Button.ButtonMode.RAW);
        mController.config(Button.ButtonID.LEFT_BUMPER, Button.ButtonMode.RAW);

        talon = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConst.intakeMaster);
        talon.setSafetyEnabled(false);

        solenoid = new DoubleSolenoid(moduleNumber, forwardChannel, reverseChannel);

        return true;
    }

    @Override
    public Command getTeleopCommand(){
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
                // Solenoid
                if (mController.get(Button.ButtonID.Y)) {
                    solenoid.set(DoubleSolenoid.Value.kForward);
                } else {
                    solenoid.set(DoubleSolenoid.Value.kReverse);
                }

                // Intake
                if (mController.get(Button.ButtonID.RIGHT_BUMPER)) {
                    talon.set(ControlMode.PercentOutput, .25);
                } else {
                    talon.set(ControlMode.PercentOutput, 0);
                }

                // Outtake
                if (mController.get(Button.ButtonID.LEFT_BUMPER)) {
                    talon.set(ControlMode.PercentOutput, -.25);
                } else {
                    talon.set(ControlMode.PercentOutput, 0);
                }
            }

            @Override
            public void stop() {
                reset();
            }
        };
    }
}
