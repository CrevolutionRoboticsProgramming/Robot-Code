package org.frc2851.robot;

import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.utilities.Vector2d;

public class Constants
{
    private static Constants mInstance = new Constants();

    public static Constants getInstance()
    {
        return mInstance;
    }

    public static Controller driver = new Controller(0);

    public final int dt_topLeftDrive = 0;
    public final int dt_topRightDrive = 1;
    public final int dt_bottomLeftDrive = 2;
    public final int dt_bottomRightDrive = 3;
    public final int dt_topLeftSwivel = 4;
    public final int dt_topRightSwivel = 5;
    public final int dt_bottomLeftSwivel = 6;
    public final int dt_bottomRightSwivel = 7;

    public final double dt_width = 2.5;
    public final double dt_length = 2.5;
    public final double dt_wheelDiameter = 1.0d / 3.0d;

    public final Vector2d dt_topLeftPosition = new Vector2d(0, 0);
    public final Vector2d dt_topRightPosition = new Vector2d(dt_width, 0);
    public final Vector2d dt_bottomLeftPosition = new Vector2d(0, dt_length);
    public final Vector2d dt_bottomRightPosition = new Vector2d(dt_width, dt_length);

    public final double dt_countsPerSwerveRotation = 4096;

    public final Vector2d robotCenter = new Vector2d(dt_width / 2, dt_length / 2);

    public final int talonTimeout = 20;
}
