package org.frc2851.vulcan;


import org.frc2851.crevolib.CrevoRobot;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.vulcan.subsystems.*;

public class Robot extends CrevoRobot
{
    public Robot()
    {
        addSubsystem(new DriveTrain(new Controller(0)));
    }
}
