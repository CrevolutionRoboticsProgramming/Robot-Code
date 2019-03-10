package org.frc2851.robot;

import edu.wpi.first.wpilibj.RobotBase;
import org.frc2851.crevolib.CrevoRobot;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.utilities.TalonSRXFactory;
import org.frc2851.robot.subsystems.*;

/**
 * Holds code unique to this year's robot
 */
public class Robot extends CrevoRobot
{
    static
    {
        TalonSRXFactory.setTalonTimeout(Constants.getInstance().talonTimeout);
    }

    private Robot()
    {
        Logger.setLogLevel(Logger.LogLevel.DEBUG);

        addSubsystem(DriveTrain.getInstance());
        addSubsystem(Elevator.getInstance());
        addSubsystem(Hatcher.getInstance());
        addSubsystem(Intake.getInstance());
        addSubsystem(RollerClaw.getInstance());
        addSubsystem(Climber.getInstance());
    }

    /**
     * Main method for code execution
     */
    public static void main(String... args)
    {
        RobotBase.startRobot(Robot::new);
    }
}