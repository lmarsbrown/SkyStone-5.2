package org.firstinspires.ftc.teamcode.Odmetry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.*;

public class Robot{
    DcMotor encLeft;
    DcMotor encRight;
    DcMotor encSide;
    public Vector3 pos = new Vector3(0,0,0);
    private Vector3 pPos = new Vector3(0,0,0);
    double wheelDia = 72;
    double countsPerRev = 2400;
    double wheelCirc;
    double encDist = 385;
    public Robot(DcMotor encLeft,DcMotor encRight,DcMotor encSide)
    {
        this.encLeft = encLeft;
        this.encRight = encRight;
        this.encSide = encSide;
        wheelCirc = wheelDia * Math.PI;
        countsPerRev = 1/countsPerRev;
        Vector3 rPosToastal = getRPosTotal();
        pPos.x = rPosToastal.r;
        pPos.y = 0.5*(rPosToastal.x-rPosToastal.y);
        pPos.r = ((rPosToastal.x+rPosToastal.y)*Math.PI*2)/(Math.PI*encDist*2);
    }
    public void relocalize()
    {
        Vector3 rPosToastal = getRPosTotal();
        double fStep = 0.5*(rPosToastal.x-rPosToastal.y)-pPos.y;
        double sStep = rPosToastal.r-pPos.x;
        double rStep = ((rPosToastal.x+rPosToastal.y)*Math.PI*2)/(Math.PI*encDist*2)-pPos.r;

        pos.x += sStep;
        pos.y += fStep;
        pos.r += rStep;
        pPos = new Vector3(rPosToastal.r,0.5*(rPosToastal.x-rPosToastal.y),((rPosToastal.x+rPosToastal.y)*Math.PI*2)/(Math.PI*encDist*2));
    }
    private Vector3 getRPosTotal()
    {
        return new Vector3(encRight.getCurrentPosition()*countsPerRev*wheelCirc,(encLeft.getCurrentPosition()*countsPerRev*wheelCirc),encSide.getCurrentPosition()*countsPerRev*wheelCirc);
    }
}
