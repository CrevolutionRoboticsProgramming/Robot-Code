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
 sets up the motor and controller used in the code
 */
public class RollerClaw extends Subsystem {
    Constants mConstants = Constants.getInstance();
    private Controller mController = Robot.operator;

    private WPI_TalonSRX _motor;

    static RollerClaw mInstance = new RollerClaw();
    boolean intake;
    boolean lastIntakeState;
    boolean outTake;
    boolean lastOutTakeState;
    private RollerClaw() {
        super("RollerClaw");
    }
    public static RollerClaw getInstance() {
        return mInstance;
    }
    /**
     initialize the motor and controller's triggers
     badlog is setup as well
     */
    @Override
    protected boolean init() {
        try {
            _motor = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.rollerClawTalon);
        } catch (TalonCommunicationErrorException e) {
            log("Could not initialize motor, roller claw init failed! Port: " + e.getPortNumber(), Logger.LogLevel.ERROR);
            return false;
        }

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
            //sets motor to zero to start
            @Override
            public boolean init() {
                _motor.set(ControlMode.PercentOutput, 0);
                return true;
            }
            /**
             shows that right trigger intakes and left trigger outtakes
             logging for intake and outtake says if its activated or deactivated
             */
            @Override
            public void update() {
                double output = .5 * mController.get(Axis.AxisID.RIGHT_TRIGGER) +
                        -.5 * mController.get(Axis.AxisID.LEFT_TRIGGER);

                _motor.set(ControlMode.PercentOutput, output);

                if(output > 0) {
                    intake = true;
                }
                else{
                    intake = false;
                }
                if (intake == true && lastIntakeState == false){
                    Logger.println("intake activated", Logger.LogLevel.DEBUG);
                }
                if (intake == false && lastIntakeState == true){
                    Logger.println("intake deactivated", Logger.LogLevel.DEBUG);
                }
                lastIntakeState = intake;

                if(output < 0) {
                    outTake = true;
                }
                else{
                    outTake = false;
                }

                if (outTake == true && lastOutTakeState == false){
                    Logger.println("outTake activated", Logger.LogLevel.DEBUG);
                }
                if (outTake == false && lastOutTakeState == true){
                    Logger.println("outTake deactivated", Logger.LogLevel.DEBUG);
                }
                lastOutTakeState = outTake;

            }

            @Override
            public void stop() {
                _motor.set(ControlMode.PercentOutput, 0);
            }
        };
    }
}