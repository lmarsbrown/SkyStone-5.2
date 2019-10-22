package org.firstinspires.ftc.teamcode.Odmetry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.*;

public class Robot_Localizer{
    //Var init
    DcMotor encLeft;
    DcMotor encRight;
    DcMotor encSide;
    public Vector3 pos = new Vector3(0,0,0);
    private Vector3 pPos = new Vector3(0,0,0);
    double wheelDia = 72;
    double countsPerRev = 2400;
    double wheelCirc;
    double encDist = 385;
    double fOffsetMulti = (Math.cos(Math.PI*0.5-Math.atan(197.50/83.89)));
    public Robot_Localizer(DcMotor encLeft,DcMotor encRight,DcMotor encSide)
    {
        //Setting vars to constructor values
        this.encLeft = encLeft;
        this.encRight = encRight;
        this.encSide = encSide;
        wheelCirc = wheelDia * Math.PI;
        countsPerRev = 1/countsPerRev;
        //Setting previous pos var to current values to reset values on restart
        Vector3 rPosToastal = getRPosTotal();
        pPos.x = rPosToastal.r;
        pPos.y = 0.5*(rPosToastal.x+rPosToastal.y);
        pPos.r = ((rPosToastal.x-rPosToastal.y)*Math.PI*2)/(Math.PI*encDist*2);
    }
    //Calculates new position based on encoder values
    public void relocalize()
    {
        //Calculating step vars
        Vector3 rPosToastal = getRPosTotal();
        double fStep = 0.5*(rPosToastal.x+rPosToastal.y)-pPos.y;
        double sStep = rPosToastal.r-pPos.x;
        double rStep = (((rPosToastal.x-rPosToastal.y)*Math.PI*2)/(Math.PI*encDist*2)-pPos.r);
        //Blending rotation and forward/sideways motion
        pos.r += rStep+0.000001;

        double arcRad = fStep/(rStep);
        Vector2 relativeArcPos = new Vector2(Math.cos(Math.PI-rStep)*arcRad+(pos.x+arcRad)+(Math.random()*0.00000001),Math.sin(Math.PI-rStep)*arcRad+pos.y+(Math.random()*0.00000001));
        pos = MyMath.rotatePoint(pos.getV2(),relativeArcPos,pos.r).getV3(pos.r);

        /*pos.x += Math.sin(pos.r)*sStep;
        pos.y -= Math.cos(pos.r)*sStep;*/
        //Rounding vars
        pos.x = Math.round(pos.x*10000000)*0.0000001;
        pos.y = Math.round(pos.y*10000000)*0.0000001;
        //Setting previous pos var
        pPos = new Vector3(rPosToastal.r,0.5*(rPosToastal.x+rPosToastal.y),((rPosToastal.x-rPosToastal.y)*Math.PI*2)/(Math.PI*encDist*2));
    }
    private Vector3 getRPosTotal()
    {
        return new Vector3(encRight.getCurrentPosition()*countsPerRev*wheelCirc,(encLeft.getCurrentPosition()*countsPerRev*wheelCirc),encSide.getCurrentPosition()*countsPerRev*wheelCirc);
    }
}