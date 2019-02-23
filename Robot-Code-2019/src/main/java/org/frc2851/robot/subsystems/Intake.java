package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;

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
    }

    public enum IntakeExtensionState
    {
        EXTENDED(DoubleSolenoid.Value.kForward), RETRACTED(DoubleSolenoid.Value.kReverse);

        private final DoubleSolenoid.Value val;

        IntakeExtensionState(DoubleSolenoid.Value val)
        {
            this.val = val;
        }
    }

    private Constants mConstants = Constants.getInstance();
    private Controller mController = Constants.driver;
    private VictorSPX intakeTalon;
    private DoubleSolenoid intakeSol;

    private static Intake mInstance = new Intake();

    private IntakeMotorState lastMotorState = IntakeMotorState.OFF;
    private IntakeExtensionState lastExtensionState = IntakeExtensionState.RETRACTED;

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

        intakeTalon = new VictorSPX(mConstants.in_talon);

        intakeSol = new DoubleSolenoid(mConstants.pcm, mConstants.in_solenoidForward, mConstants.in_solenoidReverse);

        BadLog.createTopic("Intake Percent", BadLog.UNITLESS, () -> intakeTalon.getMotorOutputPercent(), "hide", "join:Intake/Percent Outputs");
        BadLog.createTopic("Intake Voltage", "V", () -> intakeTalon.getBusVoltage(), "hide", "join:Intake/Voltage Outputs");
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
                // Poll State
                IntakeMotorState motorState = IntakeMotorState.OFF;

                IntakeExtensionState extensionState = (mController.get(Button.ButtonID.RIGHT_BUMPER)) ?
                        IntakeExtensionState.EXTENDED : IntakeExtensionState.RETRACTED;

                if (mController.get(Button.ButtonID.RIGHT_TRIGGER)) motorState = IntakeMotorState.INTAKING;
                else if (mController.get(Button.ButtonID.LEFT_TRIGGER)) motorState = IntakeMotorState.OUTTAKING;

                intakeSol.set(extensionState.val);
                intakeTalon.set(ControlMode.PercentOutput, motorState.output);

                lastMotorState = motorState;
                lastExtensionState = extensionState;
            }

            @Override
            public void stop()
            {
                reset();
            }
        };
    }
}
