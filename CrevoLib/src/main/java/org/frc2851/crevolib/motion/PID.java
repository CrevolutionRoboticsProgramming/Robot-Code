package org.frc2851.crevolib.motion;

public class PID
{
    public final double p, i, d, f;
    public PID(double p, double i, double d, double f)
    {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }
}
