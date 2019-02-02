package org.frc2851.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc2851.crevolib.drivers.TalonSRXFactory;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

public class Intake extends Subsystem {
    Constants mConst = Constants.getInstance();
    Controller mController = (mConst.singleControllerMode) ? Robot.driver : Robot.operator;
    WPI_TalonSRX talon;
    int moduleNumber = -10;
    int forwardChannel = 1;
    int reverseChannel = -1;
    DoubleSolenoid solenoid = new DoubleSolenoid(moduleNumber, forwardChannel, reverseChannel);

    public Intake(){
        super("Intake");
    }

    @Override
    protected boolean init(){
        mController.config(Button.ButtonID.Y, Button.ButtonMode.TOGGLE);
        mController.config(Button.ButtonID.RIGHT_BUMPER, Button.ButtonMode.RAW);
        mController.config(Button.ButtonID.LEFT_BUMPER, Button.ButtonMode.RAW);
        talon = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConst.intakeMaster);
        //safety for talon
        talon.setSafetyEnabled(false);
        return true;
    }

    //turns on and off pneumatic
    public void pneumatic(){
        if(mController.get(Button.ButtonID.Y)){
            solenoid.set(DoubleSolenoid.Value.kForward);
        }else{
            solenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }

    //turns motor on (both directions)
    public void motorForward(){
        if(mController.get(Button.ButtonID.RIGHT_BUMPER)){
            talon.set(ControlMode.PercentOutput, .25);
        }else{
            talon.set(ControlMode.PercentOutput, 0);
        }
    }

    //turns off motor
    public void motorReverse(){
        if(mController.get(Button.ButtonID.LEFT_BUMPER)){
            talon.set(ControlMode.PercentOutput, -.25);
        }else{
            talon.set(ControlMode.PercentOutput, 0);
        }
    }

    @Override
    public Command getTeleopCommand(){
        return new Command() {
            @Override
            public String getName() {
                return "default";
            }

            @Override
            public boolean isFinished() {
                return false;
            }

            @Override
            public boolean init() {
                return true;
            }

            @Override
            public void update() {
                pneumatic();
                motorForward();
                motorReverse();
            }

            @Override
            public void stop() {
                talon.set(ControlMode.PercentOutput, 0);
                solenoid.set(DoubleSolenoid.Value.kReverse);
            }
        };
    }
}
