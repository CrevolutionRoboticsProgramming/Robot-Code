package org.frc2851.robot.subsystems;
//when b is pressed, the screw drive actuates(moves)
import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import org.frc2851.crevolib.drivers.TalonSRXFactory;
import org.frc2851.crevolib.io.Axis;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;
import org.frc2851.robot.Constants;
// button id references for ME to use.(you can use them too if you want)
/*A(1), B(2), X(3), Y(4), START(8), SELECT(7), LEFT_BUMPER(5), RIGHT_BUMPER(6),
        LEFT_JOYSTICK(9), RIGHT_JOYSTICK(10);
        LEFT_X(0), LEFT_Y(1), RIGHT_X(4), RIGHT_Y(5), LEFT_TRIGGER(2), RIGHT_TRIGGER(3);*/

public class Climber extends Subsystem {
    // don't move LimSwitchPlaceHold, it is just a place holder line. This will be changed when the
    // limit switch is added
    private boolean LimSwitchPlaceHold=true;
    // read comment.
    private Constants mConstants = Constants.getInstance();
    private Controller stick1= Robot.driver;
    private Climber() {
        super("Climber");
    }
    private WPI_TalonSRX gorillaMaster, gorillaSlave,screwMaster;

    public boolean init() {
        stick1.config(Button.ButtonID.A, Button.ButtonMode.TOGGLE);
        stick1.config(Button.ButtonID.B, Button.ButtonMode.TOGGLE);
        gorillaMaster= TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.gorillaMaster);
        gorillaSlave= TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.gorillaSlave);
        screwMaster= TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.Screw);
        BadLog.createTopic("Climber/Master", BadLog.UNITLESS, ()->gorillaMaster.getMotorOutputPercent(),"hide","join:Climber/Percent Output");
        BadLog.createTopic("Climber/Slave", BadLog.UNITLESS, ()->gorillaSlave.getMotorOutputPercent(),"hide","join:Climber/Percent Output");
        BadLog.createTopic("Climber/Master", "Voltage:", ()->gorillaMaster.getBusVoltage(),"hide","join:Climber/Voltage Outputs");
        BadLog.createTopic("Climber/Slave", "Voltage:", ()->gorillaSlave.getBusVoltage(),"hide","join:Climber/Voltage Outputs");
        BadLog.createTopic("Climber/Master", "Amperes:", ()->gorillaMaster.getOutputCurrent(),"hide","join:Climber/Current Outputs");
        BadLog.createTopic("Climber/Slave", "Amperes:", ()->gorillaSlave.getOutputCurrent(),"hide","join:Climber/Current Outputs");
        return true;
    }
    // Makes the selected talon go to the x value
    /*
    0=gorilla arm
    1=actuating screw*/
/*private void TalonSet(double x,int select){
    if (select==0) {
        gorillaMaster.set(ControlMode.PercentOutput, x);
    }else if(select==1){
        screwMaster.set(ControlMode.PercentOutput,x);
    }
}*/
//resets all Climbing-related talons.
private void reset(){
        gorillaMaster.set(ControlMode.PercentOutput, 0);
        screwMaster.set(ControlMode.PercentOutput, 0);
    }
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
               reset();
                return true;
            }

            @Override
            public void update() {
                //controls the Arms
                if (stick1.get(Button.ButtonID.A)) {
                    gorillaMaster.set(ControlMode.PercentOutput,1.0);
                } else {
                    gorillaMaster.set(ControlMode.PercentOutput,-1.0);
                }
                if (stick1.get(Button.ButtonID.B)){
                    screwMaster.set(ControlMode.PercentOutput,1.0);
                } else {
                    screwMaster.set(ControlMode.PercentOutput,-1.0);
                }
            }


            @Override
            public void stop() {
                reset();
            }
        };
    }
}

//2 talonsSRX and one victorSPX