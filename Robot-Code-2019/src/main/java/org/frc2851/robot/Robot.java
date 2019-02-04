package org.frc2851.robot;

import edu.wpi.first.wpilibj.RobotBase;
import org.frc2851.crevolib.CrevoRobot;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.io.Axis;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.robot.subsystems.*;

public class Robot extends CrevoRobot
{
    public static Controller driver, operator;

    public Robot()
    {
        Logger.setLogLevel(Logger.LogLevel.DEBUG);
        configControllers();
        addSubsystem(DriveTrain.getInstance());
        addSubsystem(Elevator.getInstance());
        addSubsystem(Hatcher.getInstance());
    }

    private static void configControllers()
    {
        driver = new Controller(0);
        operator = new Controller(1);

        driver.config(Axis.AxisID.LEFT_Y); // Throttle
        driver.config(Axis.AxisID.RIGHT_X); // Turn
        driver.config(Axis.AxisID.RIGHT_TRIGGER); // Quick Turn

        //TODO: Config buttons for Hatcher
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}