package org.frc2851.robot;

import edu.wpi.first.wpilibj.RobotBase;
import org.frc2851.crevolib.CrevoBot;
import org.frc2851.crevolib.utilities.Logger;
import org.frc2851.crevolib.utilities.TalonSRXFactory;
import org.frc2851.robot.subsystems.ExampleSubsystem;

public class Robot extends CrevoBot
{
    /*
    Six subsystems:
        * Climber
            * A TalonSRX for the gorilla and a VictorSPX for the screw drive
        * Drive Train
            * Six TalonSRXs; three for each side
        * Elevator
            * One TalonSRX
        * Hatcher
            * Two DoubleSolenoids, one for extending and retracting the hatcher,
                one for actuating the claw
        * Intake
            * One VictorSPX and one DoubleSolenoid
        * Roller Claw
            * One VictorSPX
     */
    static
    {
        TalonSRXFactory.setTalonTimeout(Constants.getInstance().talonTimeout);
    }

    public Robot()
    {
        Logger.setLogLevel(Logger.LogLevel.DEBUG);
        enableBadLog(false);

        addSubsystem(ExampleSubsystem.getInstance());
    }

    public static void main(String... args)
    {
        RobotBase.startRobot(Robot::new);
    }
}