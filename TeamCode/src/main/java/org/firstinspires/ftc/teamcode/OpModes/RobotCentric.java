package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot_Controller;
import org.firstinspires.ftc.teamcode.Robot.Robot_Localizer;
import org.firstinspires.ftc.teamcode.Utils.Transform;


@TeleOp(name="Robot-Centric Driver", group="Iterative Opmode")
//@Disabled
public class RobotCentric extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor center = null;
    private DcMotor right = null;
    private DcMotor left = null;
    private Robot_Localizer rowboat;
    private  Robot_Controller control;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor horizontal_extender;
    DcMotor vertical_extender;
    double gp1_percent_pwr;
    double gp2_percent_pwr;
    Transform saved_robot_pos;
    Transform robot_vector;
    boolean going_to_pt;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        leftFront           = hardwareMap.get(DcMotor.class, "left_front");
        rightFront          = hardwareMap.get(DcMotor.class, "right_front");
        leftBack            = hardwareMap.get(DcMotor.class, "left_back");
        rightBack           = hardwareMap.get(DcMotor.class, "right_back");
        horizontal_extender = hardwareMap.get(DcMotor.class, "horizontal_ext");
        vertical_extender   = hardwareMap.get(DcMotor.class, "vertical_ext");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rowboat = new Robot_Localizer(leftBack,rightFront,rightBack,0.958);
        control = new Robot_Controller(rightFront,leftFront,rightBack,leftBack,rowboat);
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
        going_to_pt = false;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        rowboat.relocalize();
        robot_vector = new Transform(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);

        if(gamepad1.left_bumper) gp1_percent_pwr = 0.25;
        else gp1_percent_pwr = 1;

        if(gamepad2.left_bumper) gp2_percent_pwr = 0.25;
        else gp2_percent_pwr = 1;

        if(gamepad1.y) saved_robot_pos = rowboat.pos.clone();
        if(gamepad1.x && saved_robot_pos != null && !going_to_pt)
        {
            going_to_pt = true;
            control.gotoPoint(saved_robot_pos, true, true,0.2, (Object obj)->{going_to_pt = false; return 0;});
        }

        if(gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
            going_to_pt = false;
            control.clearGoto();
        }
        
        // Uncomment the next line for Field-Centric Driving
        robot_vector.rotate(new Transform(0, 0, 0), -rowboat.pos.r);

        if(!going_to_pt) control.setVec(robot_vector, gp1_percent_pwr);

        horizontal_extender.setPower(-gamepad2.left_stick_y * gp2_percent_pwr);

        vertical_extender.setPower(gamepad2.right_stick_y * gp2_percent_pwr);

        telemetry.addData("r",rowboat.pos.r);
        telemetry.addData("x",rowboat.pos.x);
        telemetry.addData("y",rowboat.pos.y);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
