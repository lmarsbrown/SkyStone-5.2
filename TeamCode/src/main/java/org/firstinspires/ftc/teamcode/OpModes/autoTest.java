package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot_Controller;
import org.firstinspires.ftc.teamcode.Robot.Robot_Localizer;
import org.firstinspires.ftc.teamcode.Utils.Transform;

import java.util.Vector;


@TeleOp(name="autoTest", group="Iterative Opmode")
@Disabled
public class autoTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor center = null;
    private DcMotor right = null;
    private DcMotor left = null;
    private Robot_Localizer rowboat;
    private  Robot_Controller control;
    private Vector<Transform> path = new Vector<>();
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack   = hardwareMap.get(DcMotor.class, "left_back");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rowboat = new Robot_Localizer(leftBack,rightFront,rightBack,0.958);
        control = new Robot_Controller(rightFront,leftFront,rightBack,leftBack,rowboat);

        path.add(new Transform(0,0,0));
        path.add(new Transform(0,1000,0));
        path.add(new Transform(500,0,0));

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
        control.gotoPoint(new Transform(500,-2000,0),false,true,0.3,0.00003,(Object obj)->{

            return 0;
        });
        runtime.reset();

        //control.gotoPoint(new Transform(-1000,-500,0),true);..
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        rowboat.relocalize();






        telemetry.addData("x",rowboat.pos.x);
        telemetry.addData("y",rowboat.pos.y);
        telemetry.addData("r",Math.toDegrees(rowboat.pos.r));
        telemetry.addData("telem",control.telem);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
