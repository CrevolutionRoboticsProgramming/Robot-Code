package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
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

    DigitalInput mLimitSwitch;

    private RollerClaw() {
        super("RollerClaw");
    }

    public static RollerClaw getInstance() {
        return mInstance;
    }

    private boolean isIntaking, lastIntakeState;
    private boolean isOuttaking, lastOuttakeState;

    @Override
    protected boolean init() {
        _motor = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.rollerClawTalon);
        mLimitSwitch = new DigitalInput(mConstants.rollerClawLimitSwitch);

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
                double output = .5 * mController.get(Axis.AxisID.RIGHT_TRIGGER) +
                        -.5 * mController.get(Axis.AxisID.LEFT_TRIGGER);

                _motor.set(ControlMode.PercentOutput, output);

                isIntaking = output > 0;

                if (isIntaking && !lastIntakeState){
                    log("Began Intaking", Logger.LogLevel.DEBUG);
                } else if (!isIntaking && lastIntakeState){
                    log("Stopped Intaking", Logger.LogLevel.DEBUG);
                }
                lastIntakeState = isIntaking;

                isOuttaking = output < 0;

                if (isOuttaking && !lastOuttakeState){
                    log("Began Outtaking", Logger.LogLevel.DEBUG);
                } else if (!isOuttaking && lastOuttakeState){
                    log("Stopped Outtaking", Logger.LogLevel.DEBUG);
                }
                lastOuttakeState = isOuttaking;

                if(mLimitSwitch.get()){
                    _motor.set(0.1);
                }
            }

            @Override
            public void stop() {
                _motor.set(ControlMode.PercentOutput, 0);
            }
        };
    }
}