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
import org.frc2851.robot.Robot;

/**
 * Represents the cargo intake subsystem
 */
public class Intake extends Subsystem
{
    public enum IntakeMotorState
    {
        INTAKING(1.0), OUTTAKING(-1.0), IDLE(0.0);

        private final double output;

        IntakeMotorState(double output)
        {
            this.output = output;
        }
    }

    public enum IntakeExtensionState
    {
        EXTENDED(DoubleSolenoid.Value.kForward), RETRACTED(DoubleSolenoid.Value.kReverse);

        private final DoubleSolenoid.Value value;

        IntakeExtensionState(DoubleSolenoid.Value value)
        {
            this.value = value;
        }
    }

    private Constants mConstants = Constants.getInstance();
    private Controller mController = Constants.driver;
    private VictorSPX intakeTalon;
    private DoubleSolenoid intakeSol;

    private static Intake mInstance = new Intake();

    private IntakeMotorState mMotorState = IntakeMotorState.IDLE;
    private IntakeExtensionState mExtensionState = IntakeExtensionState.RETRACTED;

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
                return "Default";
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
                if (Robot.isRunning())
                {
                    IntakeMotorState motorState = pollMotorState();
                    IntakeExtensionState extensionState = pollExtensionState();
//                log("Updated extension state: " + extensionState.name(), Logger.LogLevel.DEBUG);
                    if (mExtensionState != extensionState)
                    {

                        setCommmandGroup(setExtensionState(extensionState));
                    }
                    intakeTalon.set(ControlMode.PercentOutput, motorState.output);
                    mMotorState = motorState;
                }
            }

            @Override
            public void stop()
            {
                reset();
            }

            IntakeMotorState pollMotorState()
            {
                IntakeMotorState polledMotorState = IntakeMotorState.IDLE;
                if (mController.get(Button.ButtonID.RIGHT_TRIGGER)) polledMotorState = IntakeMotorState.INTAKING;
                else if (mController.get(Button.ButtonID.LEFT_TRIGGER)) polledMotorState = IntakeMotorState.OUTTAKING;
                return polledMotorState;
            }

            IntakeExtensionState pollExtensionState()
            {
                return (mController.get(Button.ButtonID.RIGHT_BUMPER)) ? IntakeExtensionState.EXTENDED : IntakeExtensionState.RETRACTED;
            }
        };
    }

    public Command setExtensionState(IntakeExtensionState state)
    {
        return new Command()
        {
            @Override
            public String getName()
            {
                return "SetExtensionState[" + state.name() + "]";
            }

            @Override
            public boolean isFinished()
            {
                return true;
            }

            @Override
            public boolean init()
            {
                if (state == mExtensionState) return true;
                intakeSol.set(state.value);
                mExtensionState = state;
                return true;
            }

            @Override
            public void update()
            {
            }

            @Override
            public void stop()
            {
            }
        };
    }

    public IntakeMotorState getMotorState()
    {
        return mMotorState;
    }

    public IntakeExtensionState getExtensionState()
    {
        return mExtensionState;
    }
}
