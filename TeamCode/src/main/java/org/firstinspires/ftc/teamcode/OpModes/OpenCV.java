package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Robot_Controller;
import org.firstinspires.ftc.teamcode.Robot.Robot_Localizer;
import org.firstinspires.ftc.teamcode.Utils.Transform;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraBase;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;


@Autonomous(name="OpenCV", group="Iterative Opmode")
//@Disabled
public class OpenCV extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Robot_Localizer rowboat;
    private Robot_Controller control;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor horizontal_extender;
    private DcMotor vertical_extender;

    private Servo collector_arm;
    private Servo foundation_mover;

    private CRServo outer_collector;
    private CRServo inner_collector;

    private double gp1_percent_pwr;
    private double gp2_percent_pwr;

    private Transform saved_robot_pos;
    private Transform robot_vector;

    private boolean going_to_pt;

    private DigitalChannel limit_switch_front;
    private DigitalChannel limit_switch_back;

    StonePipeline pip;

    OpenCvCamera webcam;


    @Override
    public void init() {
        /*leftFront           = hardwareMap.get(DcMotor.class, "left_front");
        rightFront          = hardwareMap.get(DcMotor.class, "right_front");
        leftBack            = hardwareMap.get(DcMotor.class, "left_back");
        rightBack           = hardwareMap.get(DcMotor.class, "right_back");
        horizontal_extender = hardwareMap.get(DcMotor.class, "horizontal_ext");
        vertical_extender   = hardwareMap.get(DcMotor.class, "vertical_ext");

        collector_arm       = hardwareMap.get(Servo.class, "collector_arm");
        foundation_mover    = hardwareMap.get(Servo.class, "Foundation_mover");

        outer_collector     = hardwareMap.get(CRServo.class, "outer_collector");
        inner_collector     = hardwareMap.get(CRServo.class, "inner_collector");

        limit_switch_back   = hardwareMap.get(DigitalChannel.class, "limit_switch1");
        limit_switch_front  = hardwareMap.get(DigitalChannel.class, "limit_switch2");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        vertical_extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontal_extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        //rowboat = new Robot_Localizer(leftBack,rightFront,rightBack,0.958);
        //control = new Robot_Controller(rightFront,leftFront,rightBack,leftBack,rowboat);

        going_to_pt = false;


        //Move collector_arm up
        //collector_arm.setPosition(0.77);







        //foundation_mover.setPosition(0);

        //WebcamName camName = hardwareMap.get(WebcamName.class,"Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        webcam.openCameraDevice();

        pip = new StonePipeline(800,0,1280,300,420);

        webcam.setPipeline(pip);


        webcam.startStreaming(1280,720, OpenCvCameraRotation.SIDEWAYS_LEFT);
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
        //rowboat.relocalize();

        /*telemetry.addData("x",rowboat.pos.x);
        telemetry.addData("y",rowboat.pos.y);
        telemetry.addData("r",rowboat.pos.r);*/
        telemetry.addData("color",pip.stonePos);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

class StonePipeline extends OpenCvPipeline
{
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */
    double stoneScreenSize;
    int stoneTop;
    int stoneBase;

    public int stonePos = 0;

    public StonePipeline(double camDist, double camX, int screenWidth,int stoneTop,int stoneBase)
    {
        stoneScreenSize =( 200/camDist)*screenWidth;
        this.stoneTop = stoneTop;
        this.stoneBase = stoneBase;
    }


    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        /*
         * Draw boxes around
         */
        Imgproc.rectangle(
                input,
                new Point(
                        stoneScreenSize*0.5,
                        stoneTop),
                new Point(
                        stoneScreenSize*1.5,
                        stoneBase),
                new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(
                input,
                new Point(
                        stoneScreenSize*1.5,
                        stoneTop),
                new Point(
                        stoneScreenSize*2.5,
                        stoneBase),
                new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(
                input,
                new Point(
                        stoneScreenSize*2.5,
                        stoneTop),
                new Point(
                        stoneScreenSize*3.5,
                        stoneBase),
                new Scalar(0, 255, 0), 4);
        Scalar s1Col = mean(input,5,stoneTop,stoneBase,(int)Math.round(stoneScreenSize*0.5),(int)Math.round(stoneScreenSize*1.5));
        double y1 = s1Col.val[0]+s1Col.val[1];

        Scalar s2Col = mean(input,5,stoneTop,stoneBase,(int)Math.round(stoneScreenSize*1.5),(int)Math.round(stoneScreenSize*2.5));
        double y2 = s2Col.val[0]+s2Col.val[1];

        Scalar s3Col = mean(input,5,stoneTop,stoneBase,(int)Math.round(stoneScreenSize*2.5),(int)Math.round(stoneScreenSize*3.5));
        double y3 = s3Col.val[0]+s3Col.val[1];

        if(y1<y2&&y1<y3)stonePos = 0;
        else if(y2<y1&&y2<y3)stonePos = 1;
        else stonePos = 2;



        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */

        return input;
    }
    private Scalar mean(Mat input, int hopDist, int top, int bottom, int left, int right)
    {
        Scalar out = new Scalar(0,0,0);

        int pixels = 0;

        for(int y = top; y < bottom; y+=1+hopDist)
        {
            for(int x = left; x < right; x+=1+hopDist)
            {
                out.val[0] += input.get(y,x)[0];
                out.val[1] += input.get(y,x)[1];
                out.val[2] += input.get(y,x)[2];
                Imgproc.rectangle(input,new Point(x,y),new Point(x,y),new Scalar(255,0,0),1);
                pixels++;
            }
        }
        return new Scalar(out.val[0]/pixels,out.val[1]/pixels,out.val[2]/pixels);
    }
}

