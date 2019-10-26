package org.firstinspires.ftc.teamcode.Utils;

public class MyMath {
    @Deprecated
    public static Transform rotatePoint(Transform pivot,Transform point,double amount)
    {
        //Creates translated and normalized point
        Transform tPoint = new Transform(point.x-pivot.x,point.y-pivot.y,point.r);
        double len = tPoint.getLength();
        tPoint.normalize();

        //rotates tPoint and scales it back up
        Transform output = new Transform((tPoint.x*Math.cos(amount)-tPoint.y* Math.sin(amount))*len,(tPoint.y*Math.cos(amount)+tPoint.x* Math.sin(amount))*len,point.r);
        return output;
    }
}
