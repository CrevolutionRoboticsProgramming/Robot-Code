//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package org.frc2851.crevolib.utilities;

public final class Vector2d
{
    public final double x;
    public final double y;

    public static Vector2d add(Vector2d var0, Vector2d var1)
    {
        return new Vector2d(var0.x + var1.x, var0.y + var1.y);
    }

    public static Vector2d sub(Vector2d var0, Vector2d var1)
    {
        return new Vector2d(var0.x - var1.x, var0.y - var1.y);
    }

    public static Vector2d componentwiseMul(Vector2d var0, Vector2d var1)
    {
        return new Vector2d(var0.x * var1.x, var0.y * var1.y);
    }

    public static Vector2d componentwiseDiv(Vector2d var0, Vector2d var1)
    {
        return new Vector2d(var0.x / var1.x, var0.y / var1.y);
    }

    public static Vector2d mul(Vector2d var0, double var1)
    {
        return new Vector2d(var0.x * var1, var0.y * var1);
    }

    public static Vector2d div(Vector2d var0, double var1)
    {
        return new Vector2d(var0.x / var1, var0.y / var1);
    }

    public static Vector2d neg(Vector2d var0)
    {
        return new Vector2d(-var0.x, -var0.y);
    }

    public double magnitude()
    {
        return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2));
    }

    public Vector2d(double var1, double var2)
    {
        this.x = var1;
        this.y = var2;
    }

    public boolean equals(Object var1)
    {
        return var1 instanceof Vector2d && ((Vector2d) var1).x == this.x && ((Vector2d) var1).y == this.y;
    }

    public String toString()
    {
        return "Vector2d{x=" + this.x + ", y=" + this.y + '}';
    }
}
