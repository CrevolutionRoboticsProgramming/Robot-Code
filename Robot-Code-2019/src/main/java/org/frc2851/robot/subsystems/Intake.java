package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
public class Intake extends Subsystem
{

    private Constants mConstants = Constants.getInstance();
    private Controller mController = (mConstants.singleControllerMode) ? Robot.driver : Robot.operator;
    private TalonSRX intakeTalon;
    private DoubleSolenoid intakeSol;

    private static Intake mInstance = new Intake();

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

    private boolean lastDeployState;
    private boolean lastIntakeState, lastOuttakeState;

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
        mController.config(Button.ButtonID.Y, Button.ButtonMode.TOGGLE);
        mController.config(Button.ButtonID.RIGHT_BUMPER, Button.ButtonMode.RAW);
        mController.config(Button.ButtonID.LEFT_BUMPER, Button.ButtonMode.RAW);

        try
        {
            intakeTalon = TalonSRXFactory.createDefaultTalonSRX(mConstants.intakeMaster);
        } catch (TalonCommunicationErrorException e)
        {
            log("Could not initialize motor, intake init failed! Port: " + e.getPortNumber(), Logger.LogLevel.ERROR);
            return false;
        }

        //TODO: Add second solenoid
        intakeSol = new DoubleSolenoid(mConstants.pcmID, mConstants.intakeForward, mConstants.intakeReverse);

        BadLog.createTopic("Intake Percent", BadLog.UNITLESS, () -> intakeTalon.getMotorOutputPercent(), "hide", "join:Intake/Percent Outputs");
        BadLog.createTopic("Intake Voltage", "V", () -> intakeTalon.getBusVoltage(), "hide", "join:Intake/Voltage Outputs");
        BadLog.createTopic("Intake Current", "A", () -> intakeTalon.getOutputCurrent(), "hide", "join:Intake/Current Outputs");
        BadLog.createTopic("Hatcher Extended", BadLog.UNITLESS, () -> intakeSol.get() == DoubleSolenoid.Value.kForward ? 1.0 : 0.0, "hide", "join:Intake/Percent Outputs");

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
                // DoubleSolenoid
                if (mController.get(Button.ButtonID.Y))
                {
                    intakeSol.set(DoubleSolenoid.Value.kForward);
                    if (!lastDeployState)
                    {
                        log("Intake Deployed", Logger.LogLevel.DEBUG);
                    }
                } else
                {
                    intakeSol.set(DoubleSolenoid.Value.kReverse);
                }
                lastDeployState = intakeSol.get() == DoubleSolenoid.Value.kForward;

                // Motor
                if (mController.get(Button.ButtonID.RIGHT_BUMPER))
                {
                    intakeTalon.set(ControlMode.PercentOutput, 1);
                    if (lastIntakeState)
                    {
                        log("Began Intaking", Logger.LogLevel.DEBUG);
                    }
                } else if (lastIntakeState)
                {
                    log("Stopped Intaking", Logger.LogLevel.DEBUG);
                }
                lastIntakeState = mController.get(Button.ButtonID.RIGHT_BUMPER);

                if (mController.get(Button.ButtonID.LEFT_BUMPER))
                {
                    intakeTalon.set(ControlMode.PercentOutput, -1);
                    if (lastIntakeState)
                    {
                        log("Began Outtaking", Logger.LogLevel.DEBUG);
                    }
                } else if (lastOuttakeState)
                {
                    log("Stopped Outtaking", Logger.LogLevel.DEBUG);
                }
                lastOuttakeState = mController.get(Button.ButtonID.LEFT_BUMPER);

                if (!mController.get(Button.ButtonID.RIGHT_BUMPER) && !mController.get(Button.ButtonID.LEFT_BUMPER))
                {
                    intakeTalon.set(ControlMode.PercentOutput, 0);
                }
            }

            @Override
            public void stop()
            {
                reset();
            }
        };
    }
}
