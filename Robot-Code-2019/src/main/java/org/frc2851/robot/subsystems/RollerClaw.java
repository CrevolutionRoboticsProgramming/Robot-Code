package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.drivers.TalonSRXFactory;
import org.frc2851.crevolib.io.Axis;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

public class RollerClaw extends Subsystem {

    Constants mConstants = Constants.getInstance();
    private Controller mController = Robot.operator;

    private WPI_TalonSRX _motor;

    static RollerClaw mInstance = new RollerClaw();

    private RollerClaw() {
        super("RollerClaw");
    }

    public static RollerClaw getInstance() {
        return mInstance;
    }

    @Override
    protected boolean init() {
        _motor = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.rollerClawTalon);

        mController.config(Axis.AxisID.RIGHT_TRIGGER);
        mController.config(Axis.AxisID.LEFT_TRIGGER);

        BadLog.createTopic("Roller Claw Percent", BadLog.UNITLESS, () -> _motor.getMotorOutputPercent(), "hide", "join:Roller Claw/Percent Outputs");
        BadLog.createTopic("Roller Claw Voltage", "V", () -> _motor.getBusVoltage(), "hide", "join:Roller Claw/Voltage Outputs");
        BadLog.createTopic("Roller Claw Current", "A", () -> _motor.getOutputCurrent(), "hide", "join:Roller Claw/Current Outputs");

        return true;
    }

    @Override
    public Command getDefaultCommand() {
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
                _motor.set(ControlMode.PercentOutput, 0);
                return true;
            }

            @Override
            public void update() {
                _motor.set(ControlMode.PercentOutput, .5 * mController.get(Axis.AxisID.RIGHT_TRIGGER));
                _motor.set(ControlMode.PercentOutput, -.5 * mController.get(Axis.AxisID.LEFT_TRIGGER));
                if (_motor.get() > 0.0) {
                    Logger.println("RollerClaw intake", Logger.LogLevel.DEBUG);
                }
                if (_motor.get() < 0.0) {
                    Logger.println("RollerClaw outtake", Logger.LogLevel.DEBUG);
                }
            }

            @Override
            public void stop() {
                _motor.set(ControlMode.PercentOutput, 0);
            }
        };
    }

}
