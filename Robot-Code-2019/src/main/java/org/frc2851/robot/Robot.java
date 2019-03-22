package org.frc2851.robot;

import edu.wpi.first.wpilibj.RobotBase;
import org.frc2851.crevolib.CrevoBot;
import org.frc2851.crevolib.utilities.Logger;
import org.frc2851.crevolib.utilities.TalonSRXFactory;
import org.frc2851.crevolib.utilities.UDPHandler;
import org.frc2851.robot.subsystems.*;

public class Robot extends CrevoBot
{
    static
    {
        TalonSRXFactory.setTalonTimeout(Constants.getInstance().talonTimeout);
    }

    private Robot()
    {
        Logger.setLogLevel(Logger.LogLevel.DEBUG);
        enableBadLog(false);

        UDPHandler.getInstance().start();

        addSubsystem(DriveTrain.getInstance());
        addSubsystem(Elevator.getInstance());
        addSubsystem(Hatcher.getInstance());
        addSubsystem(Intake.getInstance());
        addSubsystem(RollerClaw.getInstance());
        addSubsystem(Climber.getInstance());
    }

    public static void main(String... args)
    {
        RobotBase.startRobot(Robot::new);
    }
}