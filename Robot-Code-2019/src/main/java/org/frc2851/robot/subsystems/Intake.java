package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.drivers.TalonCommunicationErrorException;
import org.frc2851.crevolib.drivers.TalonSRXFactory;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

/**
 * Represents the cargo intake subsystem
 */
public class Intake extends Subsystem {

    Constants mConstants = Constants.getInstance();
    Controller mController = (mConstants.singleControllerMode) ? Robot.driver : Robot.operator;
    WPI_TalonSRX intakeTalon;
    DoubleSolenoid intakeSol;

    private static Intake mInstance = new Intake();

    /**
     * Returns the sole instance of the Intake class
     * @return The instance of the Intake class
     */
    public static Intake getInstance() {
        return mInstance;
    }

    /**
     * Initializes the Intake class with the name "Intake"
     */
    private Intake() {
        super("Intake");
    }

    /**
     * Resets the motor and solenoid
     */
    void reset() {
        intakeTalon.set(ControlMode.PercentOutput, 0);
        intakeSol.set(DoubleSolenoid.Value.kOff);
    }

    /**
     * Initializes the controller, motor, solenoid, and logging
     * @return A boolean representing whether the initialization has succeeded
     */
    @Override
    protected boolean init(){
        mController.config(Button.ButtonID.Y, Button.ButtonMode.TOGGLE);
        mController.config(Button.ButtonID.RIGHT_BUMPER, Button.ButtonMode.RAW);
        mController.config(Button.ButtonID.LEFT_BUMPER, Button.ButtonMode.RAW);

        try {
            intakeTalon = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.intakeMaster);
        } catch (TalonCommunicationErrorException e) {
            log("Could not initialize motor, intake init failed! Port: " + e.getPortNumber(), Logger.LogLevel.ERROR);
            return false;
        }
        intakeTalon.setSafetyEnabled(false);

        intakeSol = new DoubleSolenoid(mConstants.pcmID, mConstants.intakeForward, mConstants.intakeReverse);

        BadLog.createTopic("Intake Percent", BadLog.UNITLESS, () -> intakeTalon.getMotorOutputPercent(), "hide", "join:Intake/Percent Outputs");
        BadLog.createTopic("Hatcher Extended", BadLog.UNITLESS, () -> intakeSol.get() == DoubleSolenoid.Value.kForward ? 1.0 : 0.0, "hide", "join:Intake/Percent Outputs");
        BadLog.createTopic("Intake Voltage", "V", () -> intakeTalon.getBusVoltage(), "hide", "join:Intake/Voltage Outputs");
        BadLog.createTopic(" Intake Current", "A", () -> intakeTalon.getOutputCurrent(), "hide", "join:Intake/Current Outputs");

        return true;
    }

    /**
     * Returns a command representing user control over the intake
     * @return A command representing user control over the intake
     */
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

            @Override
            public boolean init() {
                reset();

                return true;
            }

            @Override
            public void update() {
                // Solenoid
                if (mController.get(Button.ButtonID.Y)) {
                    intakeSol.set(DoubleSolenoid.Value.kForward);
                    log("Intake Deployed", Logger.LogLevel.DEBUG);
                } else {
                    intakeSol.set(DoubleSolenoid.Value.kReverse);
                }

                // Intake and outtake
                if (mController.get(Button.ButtonID.RIGHT_BUMPER)) {
                    intakeTalon.set(ControlMode.PercentOutput, .25);
                    log("Intaking", Logger.LogLevel.DEBUG);
                } else if (mController.get(Button.ButtonID.LEFT_BUMPER)) {
                    intakeTalon.set(ControlMode.PercentOutput, -.25);
                    log("Outtaking", Logger.LogLevel.DEBUG);
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
