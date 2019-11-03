package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.Transform;

import static java.lang.Math.abs;
import static java.lang.Math.min;

public class RobotController {
    DcMotor rfm, lfm, rbm, lbm;
    public RobotController(DcMotor rfm,DcMotor lfm,DcMotor rbm,DcMotor lbm)
    {
        this.rfm = rfm;
        this.lfm = lfm;
        this.rbm = rbm;
        this.lbm = lbm;
    }
    public void setVec(Transform dir)
    {
        double sideMultiplierInverse                   = abs(dir.x) + abs(dir.y) + abs(dir.r);
        double sideMultiplier = min(sideMultiplierInverse, 1) / sideMultiplierInverse;

        lfm.setPower( (dir.y * sideMultiplier - dir.x * sideMultiplier + dir.r * sideMultiplier) );
        rfm.setPower((dir.y * sideMultiplier + dir.x * sideMultiplier - dir.r * sideMultiplier) );
        lbm.setPower(  (dir.y * sideMultiplier + dir.x * sideMultiplier + dir.r * sideMultiplier) );
        rbm.setPower( (dir.y * sideMultiplier - dir.x * sideMultiplier - dir.r * sideMultiplier) );
    }
}
