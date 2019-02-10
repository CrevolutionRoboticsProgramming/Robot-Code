package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.drivers.TalonCommunicationErrorException;
import org.frc2851.crevolib.drivers.TalonSRXFactory;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

/**
 * controls the Elevator Motors
 */
public class Elevator extends Subsystem
{
    public enum ElevatorControlMode { DIRECT, MOTION_MAGIC, POS_PID }

    private Constants mConst = Constants.getInstance();

    private TalonSRX mTalonMaster, mTalonSlave;
    private Controller mController = (mConst.singleControllerMode) ? Robot.driver : Robot.operator;

    private ElevatorControlMode mControlMode = ElevatorControlMode.DIRECT;
    /**
     * Creates a new mInstance for Elevator
     */
    private static Elevator mInstance = new Elevator();

    private Elevator() { super("Elevator"); }

    public static Elevator getInstance() {
        return mInstance;
    }

    /**
     * Initilizes the motors and the loggers.
     * @return
     */
    @Override
    protected boolean init()
    {
        try {
            mTalonMaster = TalonSRXFactory.createDefaultMasterTalonSRX(mConst.elevatorMaster);
            mTalonSlave = TalonSRXFactory.createPermanentSlaveTalonSRX(mConst.elevatorSlave, mTalonMaster);
        } catch (TalonCommunicationErrorException e) {
            log("Could not initialize motor, elevator init failed! Port: " + e.getPortNumber(), Logger.LogLevel.ERROR);
            return false;
        }

        BadLog.createTopicStr("Elevator/Control Mode", BadLog.UNITLESS, () -> mControlMode.name(), "hide");
        BadLog.createTopic("Elevator/Output Percent", BadLog.UNITLESS, () -> mTalonMaster.getMotorOutputPercent(), "hide");
        BadLog.createTopic("Elevator/Output Voltage Master", "V", () -> mTalonMaster.getBusVoltage(), "hide", "join:Elevator/Voltage Outputs");
        BadLog.createTopic("Elevator/Output Voltage Slave", "V", () -> mTalonSlave.getBusVoltage(), "hide", "join:Elevator/Voltage Outputs");
        BadLog.createTopic("Elevator/Output Current Master", "I", () -> mTalonMaster.getOutputCurrent(), "hide", "join:Elevator/Voltage Outputs");
        BadLog.createTopic("Elevator/Output Current Slave", "I", () -> mTalonSlave.getOutputCurrent(), "hide", "join:Elevator/Voltage Outputs");
        BadLog.createTopic("Elevator/Position", "Counts", () -> (double) mTalonMaster.getSensorCollection().getQuadraturePosition(), "hide");
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
                return false;
            }

            @Override
            public void update() {

            }

            @Override
            public void stop() {

            }
        };
    }
}
