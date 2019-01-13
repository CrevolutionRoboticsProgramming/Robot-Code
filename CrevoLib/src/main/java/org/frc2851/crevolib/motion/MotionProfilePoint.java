package org.frc2851.crevolib.motion;

import com.ctre.phoenix.motion.TrajectoryPoint;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Data structure for motion profile points. Upon construction, it converts standard units to native units.
 */
public class MotionProfilePoint
{
    public final double pos;
    public final double vel;
    public final int dt;
    public final int heading;

    /**
     * Creates a motion profile point
     * @param pos Position if feet
     * @param vel Velocity in feet per second
     * @param dt Change in time between each point
     * @param cpf Counts per feet (used for native unit conversion)
     */
    public MotionProfilePoint(double pos, double vel, int dt, double heading, double cpf)
    {
        this.pos = pathfinderToCTREPos(pos, cpf);
        this.vel = pathfinderToCTREVel(vel, cpf);
        this.dt = dt;
        // TODO: Verify heading conversion
        this.heading = (int) Math.toDegrees(heading) * 10;
    }

    private double pathfinderToCTREPos(double pos, double cpf) { return pos * cpf; }
    private double pathfinderToCTREVel(double vel, double cpf) { return (vel * cpf) / 10; }

    @Override
    public String toString()
    {
        return "MotionPoint[" + pos + ", " + vel + ", " + dt + "]";
    }
}