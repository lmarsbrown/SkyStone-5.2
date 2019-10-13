package org.firstinspires.ftc.teamcode.Odmetry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.*;

public class Robot{
    DcMotor encLeft;
    DcMotor encRight;
    DcMotor encSide;
    double x = 0;
    double y  = 0;
    double r = 0;
    public Robot(DcMotor encLeft,DcMotor encRight,DcMotor encSide)
    {
        this.encLeft = encLeft;
        this.encRight = encRight;
        this.encSide = encSide;
        Interval  posLoop = new Interval((Object obj) -> {
        this.getPosLoop();
        return 0;
    }, 10);
    }
    private Vector3 getPosLoop()
    {

        return new Vector3(0,0,0);
    }

}
