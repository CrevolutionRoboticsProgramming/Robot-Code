package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import org.frc2851.crevolib.utilities.Logger;
import org.frc2851.crevolib.io.Button;
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
    private enum RollerClawState
    {
        INTAKE(1), OUTTAKE(-1), IDLE(0.1), HOLD(0.4);

        double power;

        RollerClawState(double power)
        {
            this.power = power;
        }
    }

    private Constants mConstants = Constants.getInstance();
    private Controller mController = Constants.driver;
    private VictorSPX mVictor;
    private DigitalInput mLimitSwitch;
    private RollerClawState mState = RollerClawState.IDLE;

    private static RollerClaw mInstance = new RollerClaw();

    private RollerClaw()
    {
        super("RollerClaw");
    }

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

        mVictor = new VictorSPX(mConstants.rc_talon);
        mLimitSwitch = new DigitalInput(mConstants.rc_limitSwitch);
        mController.config(mConstants.rc_intake, Button.ButtonMode.RAW);
        mController.config(mConstants.rc_outtake, Button.ButtonMode.RAW);
        mController.config(mConstants.rc_hold, Button.ButtonMode.TOGGLE);

        BadLog.createTopic("Roller Claw Percent", BadLog.UNITLESS, () -> mVictor.getMotorOutputPercent(), "hide", "join:Roller Claw/Percent Outputs");
        BadLog.createTopic("Roller Claw Voltage", "V", () -> mVictor.getBusVoltage(), "hide", "join:Roller Claw/Voltage Outputs");
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
            boolean currentHoldState = false, lastHoldState = false;

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
                mVictor.set(ControlMode.PercentOutput, 0);
                return true;
            }

            @Override
            public void update()
            {
                if (Robot.isRunning())
                {
                    pollController();
                    mVictor.set(ControlMode.PercentOutput, mState.power);
                }
            }

            @Override
            public void stop()
            {
                mVictor.set(ControlMode.PercentOutput, 0);
            }

            public void pollController()
            {
                RollerClawState state = RollerClawState.IDLE;

                if (mController.get(mConstants.rc_outtake))
                {
                    state = RollerClawState.OUTTAKE;
                    mController.setToggleState(mConstants.rc_hold, false);
                } else if (mController.get(mConstants.rc_intake))
                {
                    state = RollerClawState.INTAKE;
                    mController.setToggleState(mConstants.rc_hold, false);
                } else if (mController.get(mConstants.rc_hold))
                {
                    state = RollerClawState.HOLD;
                }

                if (state != mState) log("Updated state: " + state.name(), Logger.LogLevel.DEBUG);

                mState = state;
            }
        };
    }

    public boolean hasCargo()
    {
        return false;
//        return mLimitSwitch.get();
    }

    public RollerClawState getState()
    {
        return mState;
    }
}
