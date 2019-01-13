package org.frc2851.robot;

public class Constants
{
    private static Constants mInstance;
    public static Constants getInstance() {
        if (mInstance == null) mInstance = new Constants();
        return mInstance;
    }

    private Constants() { }

    public final int talonLeftA = 0;
    public final int talonLeftB = 0;
    public final int talonLeftC = 0;
    public final int talonRightA = 0;
    public final int talonRightB = 0;
    public final int talonRightC = 0;

    public final int talonTimeout = 20;
}
