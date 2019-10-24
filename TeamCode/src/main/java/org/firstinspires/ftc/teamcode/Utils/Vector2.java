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
    public void rotatePoint(Vector2 pivot,double amount)
    {
        //Creates translated and normalized point
        Vector2 tPoint = new Vector2(this.x-pivot.x,this.y-pivot.y);
        double len = tPoint.getLength();
        tPoint.normalize();

        //Rotates tPoint and scales it back up Vector2 output = new Vector2((tPoint.x*Math.cos(amount)-tPoint.y* Math.sin(amount))*len,(tPoint.y*Math.cos(amount)+tPoint.x* Math.sin(amount))*len);
        this.x = (tPoint.x*Math.cos(amount)-tPoint.y* Math.sin(amount))*len+pivot.x;
        this.y = (tPoint.y*Math.cos(amount)+tPoint.x* Math.sin(amount))*len+pivot.y;

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
