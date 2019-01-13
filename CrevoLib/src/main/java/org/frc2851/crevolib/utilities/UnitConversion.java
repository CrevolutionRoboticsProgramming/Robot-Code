package org.frc2851.crevolib.utilities;

public class UnitConversion
{
    public static int countsToRotations(int counts, int cpr)
    {
        return Math.floorDiv(counts, cpr);
    }

    public static double rotationsToFeet(double rotations, double wheelDiameter)
    {
        return rotations * wheelDiameter * Math.PI;
    }
}
