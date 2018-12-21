package org.frc2851.vulcan.autons.actions;

import org.frc2851.crevolib.auton.Action;
import org.frc2851.vulcan.subsystems.DriveTrain;

public class DriveDistance implements Action
{
    DriveTrain dt = DriveTrain.getInstance();
    private final double distance;

    public DriveDistance(double distance) { this.distance = distance; }

    @Override
    public boolean init() {
        while(dt.isSubsystemActive());
        dt.setCommand(dt.driveDistanceMotionProfile(distance));
        return true;
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return !dt.isSubsystemActive();
    }

    @Override
    public void stop() {
        dt.setCommand(null);
    }
}
