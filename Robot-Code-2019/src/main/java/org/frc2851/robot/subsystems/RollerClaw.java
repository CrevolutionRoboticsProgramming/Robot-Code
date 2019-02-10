package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
public class RollerClaw extends Subsystem {
    Constants mConstants = Constants.getInstance();
    private Controller mController = Robot.operator;

    private WPI_TalonSRX _motor;

    static RollerClaw mInstance = new RollerClaw();
    boolean intake;
    boolean lastIntakeState;
    boolean outtake;
    boolean lastOuttakeState;

    /**
     * Initializes the RollerClaw class with the name "RollerClaw"
     */
    private RollerClaw() {
        super("RollerClaw");
    }

    /**
     * Returns the sole instance of the RollerClaw class
     * @return The instance of the RollerClaw class
     */
    public static RollerClaw getInstance() {
        return mInstance;
    }

    /**
     * Initializes the controller, motor, and logging
     * @return A boolean representing whether initialization has succeeded
     */
    @Override
    protected boolean init() {
        mController.config(Axis.AxisID.RIGHT_TRIGGER);
        mController.config(Axis.AxisID.LEFT_TRIGGER);

        try {
            _motor = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.rollerClawTalon);
        } catch (TalonCommunicationErrorException e) {
            log("Could not initialize motor, roller claw init failed! Port: " + e.getPortNumber(), Logger.LogLevel.ERROR);
            return false;
        }

        BadLog.createTopic("Roller Claw Percent", BadLog.UNITLESS, () -> _motor.getMotorOutputPercent(), "hide", "join:Roller Claw/Percent Outputs");
        BadLog.createTopic("Roller Claw Voltage", "V", () -> _motor.getBusVoltage(), "hide", "join:Roller Claw/Voltage Outputs");
        BadLog.createTopic("Roller Claw Current", "A", () -> _motor.getOutputCurrent(), "hide", "join:Roller Claw/Current Outputs");

        return true;
    }

    /**
     * Returns a command representing user control over the roller claw
     * @return A command representing user control over the roller claw
     */
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

                if (intake && !lastIntakeState){
                    log("Activated Intake", Logger.LogLevel.DEBUG);
                } else if (lastIntakeState){
                    log("Deactivated Intake", Logger.LogLevel.DEBUG);
                }
                lastIntakeState = intake;

                if (output < 0 && !lastOuttakeState){
                    log("Activated Outtake", Logger.LogLevel.DEBUG);
                }
                if (lastOuttakeState){
                    log("Deactivated Outtake", Logger.LogLevel.DEBUG);
                }
                lastOuttakeState = outtake;
            }

            @Override
            public void stop() {
                _motor.set(ControlMode.PercentOutput, 0);
            }
        };
    }
}