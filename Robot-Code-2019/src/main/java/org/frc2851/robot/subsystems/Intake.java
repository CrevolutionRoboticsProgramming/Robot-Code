package org.frc2851.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;

public class Intake extends Subsystem {

    public Intake(){
        super("Intake");
    }

    @Override
    protected boolean init(){

    }

    public boolean numatic(){
        TalonSRX talon1 = new TalonSRX(0);
        if(){
            talon1.set(ControlMode.PercentOutput, .25);
        }
    }

    public boolean motorIntake(){

    }

    public boolean motorOutake(){

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

            }

            @Override
            public void stop() {

            }
        }
    }
}
