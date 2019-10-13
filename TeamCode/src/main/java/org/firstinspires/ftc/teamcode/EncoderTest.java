package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Odmetry.Robot;


@TeleOp(name="Encoder Test", group="Iterative Opmode")
//@Disabled
public class EncoderTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor center = null;
    private DcMotor right = null;
    private DcMotor left = null;
    private Robot rowboat;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        center = hardwareMap.get(DcMotor.class,"right_back");
        right = hardwareMap.get(DcMotor.class,"right_front");
        left = hardwareMap.get(DcMotor.class,"left_back");
        rowboat = new Robot(left,right,center);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        rowboat.setPosInLoop();
        //telemetry.addData("center", rowboat.r);
        telemetry.addData("right", rowboat.pos.x);
        telemetry.addData("left", rowboat.pos.y);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
