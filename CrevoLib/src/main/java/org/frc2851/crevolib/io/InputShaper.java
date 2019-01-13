package org.frc2851.crevolib.io;

/**
 * Shaper interface used by the axis class. Should be used as a lambda expression.
 */
public interface InputShaper
{
    double shape(double input);
}
