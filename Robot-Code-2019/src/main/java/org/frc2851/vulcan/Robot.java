package org.frc2851.vulcan;


import org.frc2851.crevolib.CrevoRobot;
import org.frc2851.crevolib.io.Axis;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.vulcan.subsystems.*;

public class Robot extends CrevoRobot
{
    public static Controller driver, operator;

    public Robot()
    {
        configControllers();
        addSubsystem(new DriveTrain());
    }

    private static void configControllers()
    {
        driver = new Controller(0);
        operator = new Controller(1);

        driver.config(Axis.AxisID.LEFT_Y, Axis.AxisMode.INVERTED); // Throttle
        driver.config(Axis.AxisID.RIGHT_X, Axis.AxisMode.RAW); // Turn
        driver.config(Axis.AxisID.RIGHT_TRIGGER, Axis.AxisMode.RAW); // Quick Turn
    }
}
