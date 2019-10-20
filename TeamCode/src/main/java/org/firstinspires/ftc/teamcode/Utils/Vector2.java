package org.firstinspires.ftc.teamcode.Utils;

public class Vector2 {
    public double x;
    public double y;
    public Vector2(double xIn, double yIn)
    {
        x = xIn;
        y = yIn;
    }
    public void normalize()
    {
        double dist = Math.hypot(x,y);
        x/=dist;
        y/=dist;
    }
    public Vector3 getV3(double r)
    {
        return new Vector3(this.x,this.y,r);
    }
    public double getLength()
    {
        return(Math.hypot(x,y));
    }
    public Vector2 clone()
    {
        return new Vector2(x,y);
    }
}
