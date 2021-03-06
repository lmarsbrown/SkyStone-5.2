package org.firstinspires.ftc.teamcode.Utils;

public class Transform {
    public double x;
    public double y;
    public double r;
    private double zeroDec = 0.0000001;
    public Transform(double x, double y, double r)
    {
        this.x = x;
        this.y = y;
        this.r = r;
    }
    public void normalize()
    {
        double dist = Math.hypot(x,y);
        if(dist == 0)dist = zeroDec;
        x/=dist;
        y/=dist;
    }
    public void rotate(Transform pivot,double amount)
    {
        //Creates translated and normalized point+
        Transform tPoint = new Transform(this.x-pivot.x,this.y-pivot.y,this.r);
        double len = tPoint.getLength();
        if(len == 0)len = zeroDec;
        tPoint.normalize();

        //Rotates tPoint and scales it back up Transform output = new Transform((tPoint.x*Math.cos(amount)-tPoint.y* Math.sin(amount))*len,(tPoint.y*Math.cos(amount)+tPoint.x* Math.sin(amount))*len);
        this.x = (tPoint.x*Math.cos(amount)-tPoint.y* Math.sin(amount))*len+pivot.x;
        this.y = (tPoint.y*Math.cos(amount)+tPoint.x* Math.sin(amount))*len+pivot.y;

    }
    public double getLength()
    {
        return(Math.hypot(x,y));
    }
    public Transform clone()
    {
        return new Transform(x,y,r);
    }
    public Transform getAdded(Transform add)
    {
        return new Transform(this.x+add.x,this.y+add.y,this.r+add.r);
    }
    public void add(Transform vec)
    {
        this.x += vec.x;
        this.y += vec.y;
        this.r += vec.r;

    }
    public void setOrigin(Transform origin, boolean rotate)
    {
        if(rotate)this.rotate(origin,-origin.r);
        this.x -= origin.x;
        this.y -= origin.y;
    }
    public Transform getSetOrigin(Transform origin, boolean rotate)
    {
        if(rotate)this.rotate(origin,-origin.r);
        Transform output = this.clone();
        output.x -= origin.x;
        output.y -= origin.y;
        return output;
    }
    public void scale(double amount)
    {
        this.x /= amount;
        this.y /= amount;
    }
    public void stepAtPoint(Transform dir, double amount)
    {
        dir.normalize();
        dir.scale(amount);
        this.add(dir);
    }
}
