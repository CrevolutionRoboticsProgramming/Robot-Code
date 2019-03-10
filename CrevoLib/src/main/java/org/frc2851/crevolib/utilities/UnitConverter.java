package org.frc2851.crevolib.utilities;

public class UnitConverter
{
    /**
     * Returns the counts needed to go a distance
     * @param distance Distance in feet
     * @param cpf Counts per foot moved
     * @return Counts
     */
    public static int distanceToCounts(double distance, int cpf)
    {
        return (int) distance * cpf;
    }

    /**
     * Returns the counts needed to go a distance
     * @param distance Distance in feet
     * @param radius Radius of the wheel/sprocket/pulley
     * @param cpr Counts per rotation
     * @return Counts
     */
    public static int distanceToCounts(double distance, double radius, double cpr)
    {
        return distanceToCounts(distance, (int) ((1 / (2 * Math.PI * radius)) * cpr));
    }

    /**
     * Returns the provided velocity in CTRE native units (Counts per 100ms)
     * @param velocity Velocity in feet per second
     * @param cpf Counts per foot traveled
     * @return Counts per 100ms
     */
    public static int velocityToCTREVelocity(double velocity, int cpf)
    {
        return (int) velocity * cpf;
    }

    /**
     * Returns the provided velocity in CTRE native units (Counts per 100ms)
     * @param velocity Velocity in feet per second
     * @param radius Radius of the wheel/sprocket/pulley
     * @param cpr Counts per rotation
     * @return Counts per 100ms
     */
    public static int velocityToCTREVelocity(double velocity, double radius, double cpr)
    {
        return distanceToCounts(velocity,(int) ((1 / (2 * Math.PI * radius)) * cpr));
    }
}
