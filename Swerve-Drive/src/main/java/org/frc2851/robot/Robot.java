package org.frc2851.robot;

import edu.wpi.first.wpilibj.RobotBase;
import org.frc2851.crevolib.CrevoRobot;
import org.frc2851.crevolib.utilities.Logger;
import org.frc2851.crevolib.utilities.TalonSRXFactory;
import org.frc2851.crevolib.utilities.UDPHandler;

public class Robot extends CrevoRobot
{
    static
    {
        TalonSRXFactory.setTalonTimeout(Constants.getInstance().talonTimeout);
    }

    private Robot()
    {
        Logger.setLogLevel(Logger.LogLevel.DEBUG);
        //enableBadLog(false);

        UDPHandler.getInstance().start();

        addSubsystem(SwerveDrive.getInstance());
    }

    public static void main(String... args)
    {
        RobotBase.startRobot(Robot::new);
    }
}