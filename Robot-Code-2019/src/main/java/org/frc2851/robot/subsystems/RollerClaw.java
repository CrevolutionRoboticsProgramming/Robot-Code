package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private enum IntakeState
    {
        INTAKE(0.75), OUTTAKE(-0.75), IDLE(0), HOLD(0.1);

        double power;
        IntakeState(double power) {
            this.power = power;
        }
    }

    private Constants mConstants = Constants.getInstance();
    private Controller mController = Constants.driver;

    private VictorSPX _motor;
    private DigitalInput mLimitSwitch = new DigitalInput(mConstants.rc_limitSwitch);

    private static RollerClaw mInstance = new RollerClaw();

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

        _motor = new VictorSPX(mConstants.rc_talon);

//        mLimitSwitch = new DigitalInput(mConstants.rc_limitSwitch);

        mController.config(Axis.AxisID.RIGHT_TRIGGER);
        mController.config(Axis.AxisID.LEFT_TRIGGER);

        BadLog.createTopic("Roller Claw Percent", BadLog.UNITLESS, () -> _motor.getMotorOutputPercent(), "hide", "join:Roller Claw/Percent Outputs");
        BadLog.createTopic("Roller Claw Voltage", "V", () -> _motor.getBusVoltage(), "hide", "join:Roller Claw/Voltage Outputs");
//        BadLog.createTopic("Roller Claw Current", "A", () -> _motor.getOutputCurrent(), "hide", "join:Roller Claw/Current Outputs");

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
            IntakeState lastState = IntakeState.IDLE;

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

                if (Robot.isRunning())
                {
                    IntakeState state = IntakeState.IDLE;

                    if (mController.get(mConstants.rc_outtake)) state = IntakeState.OUTTAKE;
                    else if (mController.get(mConstants.rc_intake)) state = IntakeState.INTAKE;
                    else if (mLimitSwitch.get()) state = IntakeState.HOLD;

//                    if (state != lastState) log("Updated state: " + state.name(), Logger.LogLevel.DEBUG);
                    _motor.set(ControlMode.PercentOutput, state.power);
                    lastState = state;
                }
            }

            @Override
            public void stop()
            {
                _motor.set(ControlMode.PercentOutput, 0);
            }
        };
    }

    public boolean hasCargo()
    {
        return mLimitSwitch.get();
    }
}
