package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.drivers.TalonCommunicationErrorException;
import org.frc2851.crevolib.drivers.TalonSRXFactory;
import org.frc2851.crevolib.io.Axis;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

/**
 * Represents the roller claw subsystem
 */
public class RollerClaw extends Subsystem
{
    private Constants mConstants = Constants.getInstance();
    private Controller mController = Robot.operator;

    private TalonSRX _motor;

    private static RollerClaw mInstance = new RollerClaw();

    // Positioned at the back of the roller claw; if pressed, we have a cargo, so we run the motors at 0.1 speed to keep it in
    private DigitalInput mLimitSwitch;

    private boolean isIntaking, lastIntakeState;
    private boolean isOuttaking, lastOuttakeState;

    private RollerClaw()
    {
        super("RollerClaw");
    }

    /**
     * Returns the sole instance of the RollerClaw class
     *
     * @return The instance of the RollerClaw class
     */
    public static RollerClaw getInstance()
    {
        return mInstance;
    }

    /**
     * Initializes the controller, motor, and logging
     *
     * @return A boolean representing whether initialization has succeeded
     */
    @Override
    protected boolean init()
    {
        try
        {
            _motor = TalonSRXFactory.createDefaultMasterTalonSRX(mConstants.rollerClawTalon);
        } catch (TalonCommunicationErrorException e)
        {
            log("Could not initialize motor, roller claw init failed! Port: " + e.getPortNumber(), Logger.LogLevel.ERROR);
            return false;
        }

        mLimitSwitch = new DigitalInput(mConstants.rollerClawLimitSwitch);

        mController.config(Axis.AxisID.RIGHT_TRIGGER);
        mController.config(Axis.AxisID.LEFT_TRIGGER);

        BadLog.createTopic("Roller Claw Percent", BadLog.UNITLESS, () -> _motor.getMotorOutputPercent(), "hide", "join:Roller Claw/Percent Outputs");
        BadLog.createTopic("Roller Claw Voltage", "V", () -> _motor.getBusVoltage(), "hide", "join:Roller Claw/Voltage Outputs");
        BadLog.createTopic("Roller Claw Current", "A", () -> _motor.getOutputCurrent(), "hide", "join:Roller Claw/Current Outputs");

        return true;
    }

    /**
     * Returns a command representing user control over the roller claw
     *
     * @return A command representing user control over the roller claw
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
                _motor.set(ControlMode.PercentOutput, 0);
                return true;
            }

            @Override
            public void update()
            {
                double output = .5 * mController.get(Axis.AxisID.RIGHT_TRIGGER) +
                        -.5 * mController.get(Axis.AxisID.LEFT_TRIGGER);

                _motor.set(ControlMode.PercentOutput, output);

                isIntaking = output > 0;

                if (isIntaking && !lastIntakeState)
                {
                    log("Began Intaking", Logger.LogLevel.DEBUG);
                } else if (!isIntaking && lastIntakeState)
                {
                    log("Stopped Intaking", Logger.LogLevel.DEBUG);
                }
                lastIntakeState = isIntaking;

                isOuttaking = output < 0;

                if (isOuttaking && !lastOuttakeState)
                {
                    log("Began Outtaking", Logger.LogLevel.DEBUG);
                } else if (!isOuttaking && lastOuttakeState)
                {
                    log("Stopped Outtaking", Logger.LogLevel.DEBUG);
                }
                lastOuttakeState = isOuttaking;

                if (mLimitSwitch.get())
                {
                    _motor.set(ControlMode.PercentOutput, 0.1);
                }
            }

            @Override
            public void stop()
            {
                _motor.set(ControlMode.PercentOutput, 0);
            }
        };
    }
}