package org.firstinspires.ftc.teamcode.Utils;

public class Vector3 {
    public double x;
    public double y;
    public double r;
    public Vector3(double xIn, double yIn, double rIn)
    {
        x = xIn;
        y = yIn;
        r = rIn;
    }
    public void normalize()
    {
        double dist = Math.sqrt(x*x+y*y);
        x/=dist;
        y/=dist;
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
