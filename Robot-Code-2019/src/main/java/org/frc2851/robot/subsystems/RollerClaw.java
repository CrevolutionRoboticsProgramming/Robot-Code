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
    /**
     booleans for logging
     */
    boolean intake;
    boolean lastIntakeState;
    boolean outTake;
    boolean lastOutTakeState;


    public static RollerClaw getInstance() {
        return mInstance;
    }

    @Override
    protected boolean init() {
        /**
         initialize the motor and controller's triggers
         badlog is setup as well
         */
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
                /**
                 shows that right trigger intakes and left trigger outtakes
                  */
                double output = .5 * mController.get(Axis.AxisID.RIGHT_TRIGGER) +
                        -.5 * mController.get(Axis.AxisID.LEFT_TRIGGER);

                _motor.set(ControlMode.PercentOutput, output);
                /**
                 logging for intake says if its activated or deactivated
                 */
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
                /**
                 logging for outtake says if its activated or deactivated
                  */
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