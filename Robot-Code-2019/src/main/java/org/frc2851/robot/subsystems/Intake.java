package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
public class Intake extends Subsystem
{
    public enum IntakeMotorState
    {
        INTAKING(1.0), OUTTAKING(-1.0), OFF(0.0);

        private final double output;

        IntakeMotorState(double output)
        {
            this.output = output;
        }

        public double getOutput()
        {
            return output;
        }
    }

    private Constants mConstants = Constants.getInstance();
    private Controller mController = (mConstants.singleControllerMode) ? Robot.driver : Robot.operator;
    private TalonSRX intakeTalon;
    private DoubleSolenoid intakeSol;

    private static Intake mInstance = new Intake();

    private IntakeMotorState motorState = IntakeMotorState.OFF,
        lastMotorState = IntakeMotorState.OFF;
    private DoubleSolenoid.Value solenoidState = DoubleSolenoid.Value.kOff,
        lastSolenoidState = DoubleSolenoid.Value.kOff;

    /**
     * Returns the sole instance of the Intake class
     *
     * @return The instance of the Intake class
     */
    public static Intake getInstance()
    {
        return mInstance;
    }

    /**
     * Initializes the Intake class with the name "Intake"
     */
    private Intake()
    {
        super("Intake");
    }

    /**
     * Resets the motor and solenoid
     */
    private void reset()
    {
        intakeTalon.set(ControlMode.PercentOutput, 0);
        intakeSol.set(DoubleSolenoid.Value.kOff);
    }

    /**
     * Initializes the controller, motor, solenoid, and logging
     *
     * @return A boolean representing whether the initialization has succeeded
     */
    @Override
    protected boolean init()
    {
        mController.config(mConstants.in_extend, Button.ButtonMode.TOGGLE);
        mController.config(mConstants.in_intake, Button.ButtonMode.RAW);
        mController.config(mConstants.in_outake, Button.ButtonMode.RAW);

        try
        {
            intakeTalon = TalonSRXFactory.createDefaultTalonSRX(mConstants.in_talon);
        } catch (TalonCommunicationErrorException e)
        {
            log("Could not initialize motor, intake init failed! Port: " + e.getPortNumber(), Logger.LogLevel.ERROR);
            return false;
        }

        intakeSol = new DoubleSolenoid(mConstants.pcm, mConstants.in_solenoidForward, mConstants.in_solenoidReverse);

        BadLog.createTopic("Intake Percent", BadLog.UNITLESS, () -> intakeTalon.getMotorOutputPercent(), "hide", "join:Intake/Percent Outputs");
        BadLog.createTopic("Intake Voltage", "V", () -> intakeTalon.getBusVoltage(), "hide", "join:Intake/Voltage Outputs");
        BadLog.createTopic("Intake Current", "A", () -> intakeTalon.getOutputCurrent(), "hide", "join:Intake/Current Outputs");
        BadLog.createTopic("Intake Extended", BadLog.UNITLESS, () -> intakeSol.get() == DoubleSolenoid.Value.kForward ? 1.0 : 0.0, "hide", "join:Intake/Percent Outputs");

        return true;
    }

    /**
     * Returns a command representing user control over the intake
     *
     * @return A command representing user control over the intake
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
                // Double Solenoid
                if(mController.get(mConstants.in_extend))
                {
                    solenoidState = DoubleSolenoid.Value.kForward;
                } else
                {
                    solenoidState = DoubleSolenoid.Value.kReverse;
                }

                // Motor
                if (mController.get(mConstants.in_intake))
                {
                    motorState = IntakeMotorState.INTAKING;
                } else if (mController.get(mConstants.in_outake))
                {
                    motorState = IntakeMotorState.OUTTAKING;
                } else
                {
                    motorState = IntakeMotorState.OFF;
                }

                intakeSol.set(solenoidState);
                intakeTalon.set(ControlMode.PercentOutput, motorState.getOutput());

                if (motorState != lastMotorState)
                {
                    log("Motor set to " + motorState.toString(), Logger.LogLevel.DEBUG);
                }
                if (solenoidState != lastSolenoidState)
                {
                    log("Solenoid set to " + solenoidState.toString(), Logger.LogLevel.DEBUG);
                }

                lastMotorState = motorState;
                lastSolenoidState = solenoidState;
            }

            @Override
            public void stop()
            {
                reset();
            }
        };
    }
}
