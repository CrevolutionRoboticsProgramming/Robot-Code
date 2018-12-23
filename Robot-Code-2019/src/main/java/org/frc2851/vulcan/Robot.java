package org.frc2851.vulcan;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import org.frc2851.crevolib.CrevoRobot;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.io.Axis;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.vulcan.subsystems.*;

public class Robot extends CrevoRobot
{
    public static Controller driver, operator;

    public Robot()
    {
        Logger.setLogLevel(Logger.LogLevel.DEBUG);
        configControllers();
        addSubsystem(DriveTrain.getInstance());
    }

    private static void configControllers()
    {
        driver = new Controller(0);
        operator = new Controller(1);

        driver.config(Axis.AxisID.LEFT_Y); // Throttle
        driver.config(Axis.AxisID.RIGHT_X); // Turn
        driver.config(Axis.AxisID.RIGHT_TRIGGER); // Quick Turn
    }
}
