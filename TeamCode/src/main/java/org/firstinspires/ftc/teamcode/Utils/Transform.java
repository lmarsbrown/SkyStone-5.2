package org.firstinspires.ftc.teamcode.Utils;

public class Transform extends Vector2 {
    public double x;
    public double y;
    public double r;
    public Transform(double x, double y, double r)
    {
        super(x,y);
        this.r = r;
    }
    public double getLength()
    {
        return(Math.hypot(x,y));
    }
    public Vector2 getV2()
    {
        return new Vector2(x,y);
    }
}
