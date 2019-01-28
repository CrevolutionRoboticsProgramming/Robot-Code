package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.drivers.TalonSRXFactory;
import org.frc2851.crevolib.io.Axis;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

public class Elevator extends Subsystem
{
    public enum ElevatorControlMode
    {
        DIRECT(ControlMode.PercentOutput),
        MOTION_MAGIC(ControlMode.MotionMagic),
        POS_PID(ControlMode.Position),
        CURRENT(ControlMode.Current);

        private final ControlMode controlMode;
        ElevatorControlMode(ControlMode controlMode) {
            this.controlMode = controlMode;
        }

        ControlMode getMode() { return controlMode; }
    }

    public enum ElevatorPosition
    {
        LOW_HATCH(0), MID_HATCH(0), HIGH_HATCH(0),
        LOW_CARGO(0), MID_CARGO(0), HIGH_CARGO(0);

        private final int pos;
        ElevatorPosition(int pos) {
            this.pos = pos;
        }
        public int getPos() { return pos; }
    }

    private Constants mConst = Constants.getInstance();

    private TalonSRX mTalonMaster, mTalonSlave;
    private Controller mController = (mConst.singleControllerMode) ? Robot.driver : Robot.operator;

    private ElevatorControlMode mControlMode = ElevatorControlMode.DIRECT;

    private boolean mDefaultMotorLock;

    private static Elevator mInstance = new Elevator();

    private Elevator() { super("Elevator"); }

    public static Elevator getInstance() {
        return mInstance;
    }

    @Override
    protected boolean init()
    {
        mTalonMaster = TalonSRXFactory.createDefaultMasterTalonSRX(mConst.elevatorMaster);
        mTalonSlave = TalonSRXFactory.createPermanentSlaveTalonSRX(mConst.elevatorSlave, mTalonMaster);

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
    public Command getTeleopCommand() {
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
            public void update()
            {
                switch (mControlMode)
                {
                    // TODO: Implement a hold method
                    case DIRECT:
                        mTalonMaster.set(ControlMode.PercentOutput, mController.get(Axis.AxisID.RIGHT_Y));
                        break;
                    case CURRENT:
                        break;
                    case POS_PID:
                        break;
                    case MOTION_MAGIC:

                        break;
                    default:
                        log("Elevator mode is null??", Logger.LogLevel.ERROR);
                        break;
                }
            }

            @Override
            public void stop() {

            }
        };
    }

    public Command moveToPosition(ElevatorControlMode controlMode)
    {
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

    public Command moveForTime(double time, double power)
    {
        return new Command() {
            @Override
            public String getName() {
                return "MoveForTime[" + time + "s, " + power + "]";
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

    public void configController(Controller controller)
    {
        /*
         * A - Move Elevator to Mid Cargo
         * B - Move Elevator to Low Cargo
         * X - Move Elevator to Mid Hatch
         * Y - Move Elevator to High Hatch
         * Right Y - Manual elevator control
         */

        controller.config(Button.ButtonID.A, Button.ButtonMode.RAW);
        controller.config(Button.ButtonID.B, Button.ButtonMode.RAW);
        controller.config(Button.ButtonID.X, Button.ButtonMode.RAW);
        controller.config(Button.ButtonID.Y, Button.ButtonMode.RAW);
        controller.config(Axis.AxisID.RIGHT_Y, x -> -x * mConst.elevatorRawMultiplier);
    }
}
