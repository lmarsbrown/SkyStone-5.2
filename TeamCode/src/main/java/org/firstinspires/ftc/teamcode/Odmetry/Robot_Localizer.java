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
        double rStep = (((rPosToastal.x-rPosToastal.y)*Math.PI*2)/(Math.PI*encDist*2)-pPos.r)/*fOffsetMulti*/;
        //Blending rotation and forward/sideways motion
        double arcRad = fStep/(pos.r+rStep);
        pos.x += sStep;
        pos.y += fStep;
        pos.r += rStep;
        //Setting previous pos var
        pPos = new Vector3(rPosToastal.r,0.5*(rPosToastal.x+rPosToastal.y),((rPosToastal.x-rPosToastal.y)*Math.PI*2)/(Math.PI*encDist*2));
    }
    private Vector3 getRPosTotal()
    {
        return new Vector3(encRight.getCurrentPosition()*countsPerRev*wheelCirc,(encLeft.getCurrentPosition()*countsPerRev*wheelCirc),encSide.getCurrentPosition()*countsPerRev*wheelCirc);
    }
}
