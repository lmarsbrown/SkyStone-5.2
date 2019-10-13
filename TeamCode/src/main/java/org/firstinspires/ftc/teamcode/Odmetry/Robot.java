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
    double pDist;
    public Robot(DcMotor encLeft,DcMotor encRight,DcMotor encSide)
    {
        this.encLeft = encLeft;
        this.encRight = encRight;
        this.encSide = encSide;
        wheelCirc = wheelDia * Math.PI;
        countsPerRev = 1/countsPerRev;
        pPos.x = encSide.getCurrentPosition()*countsPerRev*wheelCirc;
        pPos.y = 0.5*((encRight.getCurrentPosition()*countsPerRev*wheelCirc)+(encLeft.getCurrentPosition()*countsPerRev*wheelCirc));
    }
    public void setPosInLoop()
    {
        Vector3 rPosToastal = getRPosTotal();
        pos.x += rPosToastal.r-pPos.x;
        pos.y += 0.5*(rPosToastal.x-rPosToastal.y)-pPos.y;
        pPos = new Vector3(rPosToastal.r,0.5*(rPosToastal.x-rPosToastal.y),0);
    }
    private Vector3 getRPosTotal()
    {
        return new Vector3(encRight.getCurrentPosition()*countsPerRev*wheelCirc,(encLeft.getCurrentPosition()*countsPerRev*wheelCirc),encSide.getCurrentPosition()*countsPerRev*wheelCirc);
    }
}
