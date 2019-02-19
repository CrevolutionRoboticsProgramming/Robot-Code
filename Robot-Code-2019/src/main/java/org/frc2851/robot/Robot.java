package org.frc2851.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc2851.crevolib.CrevoRobot;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.io.Axis;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.robot.subsystems.*;

/**
 * Holds code unique to this year's robot
 */
public class Robot extends CrevoRobot
{

    public static Controller driver, operator;
    private SendableChooser driveModeSelector = new SendableChooser();


    /**
     * Initializes the Robot class
     */
    private Robot()
    {
        Logger.setLogLevel(Logger.LogLevel.DEBUG);
        configControllers();
        //addSubsystem(DriveTrain.getInstance());
        addSubsystem(Elevator.getInstance());
        //addSubsystem(Hatcher.getInstance());
        //addSubsystem(Intake.getInstance());
        for (DriveTrain.DriveControlMode m : DriveTrain.DriveControlMode.values()) driveModeSelector.addOption(m.name(), m);
        driveModeSelector.setDefaultOption("FPS", DriveTrain.DriveControlMode.FPS);
        SmartDashboard.putData("Drive Selection", driveModeSelector);
    }

    public void teleopInit()
    {
        DriveTrain.setDriveMode((DriveTrain.DriveControlMode) driveModeSelector.getSelected());
    }

    /**
     * Configures the controllers
     */
    private static void configControllers()
    {
        driver = new Controller(0);
        operator = new Controller(1);

        driver.config(Axis.AxisID.LEFT_Y); // Throttle
        driver.config(Axis.AxisID.RIGHT_X); // Turn
        driver.config(Axis.AxisID.RIGHT_TRIGGER); // Quick Turn

        //TODO: Config buttons for Hatcher
    }

    /**
     * Main method for code execution
     */
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}