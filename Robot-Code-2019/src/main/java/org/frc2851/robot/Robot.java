package org.frc2851.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc2851.crevolib.CrevoRobot;
import org.frc2851.crevolib.Logger;
import org.frc2851.robot.subsystems.*;

/**
 * Holds code unique to this year's robot
 */
public class Robot extends CrevoRobot
{

    private SendableChooser driveModeSelector = new SendableChooser();


    /**
     * Initializes the Robot class
     */
    private Robot()
    {
        Logger.setLogLevel(Logger.LogLevel.DEBUG);

        addSubsystem(DriveTrain.getInstance());
        addSubsystem(Elevator.getInstance());
        addSubsystem(Hatcher.getInstance());
        addSubsystem(Intake.getInstance());
        addSubsystem(RollerClaw.getInstance());
        addSubsystem(Climber.getInstance());

        for (DriveTrain.DriveControlMode m : DriveTrain.DriveControlMode.values())
            driveModeSelector.addOption(m.name(), m);
        driveModeSelector.setDefaultOption("FPS", DriveTrain.DriveControlMode.FPS);
        SmartDashboard.putData("Drive Selection", driveModeSelector);
    }

    public void teleopInit()
    {
        DriveTrain.setDriveMode((DriveTrain.DriveControlMode) driveModeSelector.getSelected());
    }

    /**
     * Main method for code execution
     */
    public static void main(String... args)
    {
        RobotBase.startRobot(Robot::new);
    }
}