package org.frc2851.crevolib.utilities;

public final class Vector2D
{
    public final double x;
    public final double y;

    public static Vector2D add(Vector2D var0, Vector2D var1)
    {
        return new Vector2D(var0.x + var1.x, var0.y + var1.y);
    }

    public static Vector2D sub(Vector2D var0, Vector2D var1)
    {
        return new Vector2D(var0.x - var1.x, var0.y - var1.y);
    }

    public static Vector2D componentwiseMul(Vector2D var0, Vector2D var1)
    {
        return new Vector2D(var0.x * var1.x, var0.y * var1.y);
    }

    public static Vector2D componentwiseDiv(Vector2D var0, Vector2D var1)
    {
        return new Vector2D(var0.x / var1.x, var0.y / var1.y);
    }

    public static Vector2D mul(Vector2D var0, double var1)
    {
        return new Vector2D(var0.x * var1, var0.y * var1);
    }

    public static Vector2D div(Vector2D var0, double var1)
    {
        return new Vector2D(var0.x / var1, var0.y / var1);
    }

    public static Vector2D neg(Vector2D var0)
    {
        return new Vector2D(-var0.x, -var0.y);
    }

    public double magnitude()
    {
        return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2));
    }

    public Vector2D(double var1, double var2)
    {
        this.x = var1;
        this.y = var2;
    }

    public boolean equals(Object var1)
    {
        return var1 instanceof Vector2D && ((Vector2D) var1).x == this.x && ((Vector2D) var1).y == this.y;
    }

    public String toString()
    {
        return "Vector2D{x=" + this.x + ", y=" + this.y + '}';
    }
}
