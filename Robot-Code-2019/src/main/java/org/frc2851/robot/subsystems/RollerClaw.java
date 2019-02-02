package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import org.frc2851.crevolib.drivers.TalonSRXFactory;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

public class RollerClaw extends Subsystem {
    private Controller mController = Robot.operator;
    Joystick stick = new Joystick(0);


    private WPI_TalonSRX _Motor;

    RollerClaw mInstance = new RollerClaw();

    Constants mConstants = Constants.getInstance();


    private RollerClaw() {
        super("RollerClaw");
    }

    @Override
    protected boolean init() {

        _Motor = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.rollerClawTalon);
        BadLog.createTopic("Roller-Claw Percent", BadLog.UNITLESS, () -> _Motor.getMotorOutputPercent(), "hide", "join:Roller-Claw/Percent Outputs");

        BadLog.createTopic("Roller-Claw Voltage", "V", () -> _Motor.getBusVoltage(), "hide", "join:Roller-Claw/Voltage Outputs");

        BadLog.createTopic("Roller-Claw Current", "A", () -> _Motor.getOutputCurrent(), "hide", "join:Roller-Claw/Current Outputs");
        return true;
    }


    @Override
    public Command getTeleopCommand() {
        return new Command() {

            @Override
            public String getName() {
                return null;
            }

            @Override
            public boolean isFinished() {
                return false;
            }

            @Override
            public boolean init() {
              _Motor.set(ControlMode.PercentOutput,(0));
              return true;
            }

            @Override
            public void update() {
                _Motor.set(ControlMode.PercentOutput,.5 * stick.getRawAxis(2));
                _Motor.set(ControlMode.PercentOutput,.5 * -stick.getRawAxis(3));


            }

            @Override
            public void stop() {

            }
        };
    }
}
