package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.drivers.TalonSRXFactory;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

/**declares the motor used and the controller amongst other stuff needed in the code*/
public class Intake extends Subsystem {

    Constants mConst = Constants.getInstance();
    Controller mController = (mConst.singleControllerMode) ? Robot.driver : Robot.operator;
    WPI_TalonSRX intakeTalon;
    int moduleNumber = -10;
    int forwardChannel = 1;
    int reverseChannel = -1;
    DoubleSolenoid intakeSol;

    private static Intake mInstance = new Intake();

    public static Intake getInstance() {
        return mInstance;
    }
    private Intake() {
        super("Intake");
    }

    void reset() {
        intakeTalon.set(ControlMode.PercentOutput, 0);
        intakeSol.set(DoubleSolenoid.Value.kOff);
    }
    /**
     initializes the buttons and motor and solenoid
      */
    @Override
    protected boolean init(){
        mController.config(Button.ButtonID.Y, Button.ButtonMode.TOGGLE);
        mController.config(Button.ButtonID.RIGHT_BUMPER, Button.ButtonMode.RAW);
        mController.config(Button.ButtonID.LEFT_BUMPER, Button.ButtonMode.RAW);

        intakeTalon = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConst.intakeMaster);
        intakeTalon.setSafetyEnabled(false);

        intakeSol = new DoubleSolenoid(moduleNumber, forwardChannel, reverseChannel);

        return true;
    }

    @Override
    public Command getDefaultCommand(){
        return new Command() {
            @Override
            public String getName() {
                return "Teleop";
            }

            @Override
            public boolean isFinished() {
                return false;
            }
            /**begins badlog*/
            @Override
            public boolean init() {
                reset();
                BadLog.createTopic("Intake Percent", BadLog.UNITLESS, () -> intakeTalon.getMotorOutputPercent(), "hide", "join:Intake/Percent Outputs");
                BadLog.createTopic("Hatcher Extended", BadLog.UNITLESS, () -> intakeSol.get() == DoubleSolenoid.Value.kForward ? 1.0 : 0.0, "hide", "join:Intake/Percent Outputs");

                BadLog.createTopic("Intake Voltage", "V", () -> intakeTalon.getBusVoltage(), "hide", "join:Intake/Voltage Outputs");

                BadLog.createTopic(" Intake Current", "A", () -> intakeTalon.getOutputCurrent(), "hide", "join:Intake/Current Outputs");

                return true;
            }
            /**
            *sets motors and solenoid to their respective buttons and logs it
             */
            @Override
            public void update() {
                // Solenoid
                if (mController.get(Button.ButtonID.Y)) {
                    intakeSol.set(DoubleSolenoid.Value.kForward);
                    Logger.println("Intake Deployed", Logger.LogLevel.DEBUG);
                } else {
                    intakeSol.set(DoubleSolenoid.Value.kReverse);
                }

                // Intake
                if (mController.get(Button.ButtonID.RIGHT_BUMPER)) {
                    intakeTalon.set(ControlMode.PercentOutput, .25);
                    Logger.println("Intake In ", Logger.LogLevel.DEBUG);
                }
                // OutTake
                else if (mController.get(Button.ButtonID.LEFT_BUMPER)) {
                    intakeTalon.set(ControlMode.PercentOutput, -.25);
                    Logger.println("Intake Out", Logger.LogLevel.DEBUG);
                }
                else {
                    intakeTalon.set(ControlMode.PercentOutput, 0);
                }


            }

            @Override
            public void stop() {
                reset();
            }
        };
    }
}
