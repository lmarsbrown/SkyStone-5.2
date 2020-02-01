/*
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.MotorController;
import org.firstinspires.ftc.teamcode.Robot.PIDController;
import org.firstinspires.ftc.teamcode.Robot.Robot_Controller;
import org.firstinspires.ftc.teamcode.Robot.Robot_Localizer;
import org.firstinspires.ftc.teamcode.Robot.StonePipeline;
import org.firstinspires.ftc.teamcode.Utils.Interval;
import org.firstinspires.ftc.teamcode.Utils.Lambda;
import org.firstinspires.ftc.teamcode.Utils.Transform;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
//@Disabled
public class FullRedAuto extends OpMode {
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
    private Servo right_stone_collector;
    private Servo right_stone_collector_arm;
    private Servo left_stone_collector;
    private Servo left_stone_collector_arm;

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

    private MotorController vertCont;


    @Override
    public void init() {
        leftFront           = hardwareMap.get(DcMotor.class, "left_front");
        rightFront          = hardwareMap.get(DcMotor.class, "right_front");
        leftBack            = hardwareMap.get(DcMotor.class, "left_back");
        rightBack           = hardwareMap.get(DcMotor.class, "right_back");
        horizontal_extender = hardwareMap.get(DcMotor.class, "horizontal_ext");
        vertical_extender   = hardwareMap.get(DcMotor.class, "vertical_ext");

        collector_arm       = hardwareMap.get(Servo.class, "collector_arm");
        foundation_mover    = hardwareMap.get(Servo.class, "Foundation_mover");
        right_stone_collector = hardwareMap.get(Servo.class, "right_stone_collector");
        right_stone_collector_arm = hardwareMap.get(Servo.class, "right_stone_collector_arm");
        left_stone_collector = hardwareMap.get(Servo.class, "left_stone_collector");
        left_stone_collector_arm = hardwareMap.get(Servo.class, "left_stone_collector_arm");

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
        horizontal_extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vertical_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical_extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rowboat = new Robot_Localizer(leftBack,rightFront,rightBack,0.958);
        control = new Robot_Controller(rightFront,leftFront,rightBack,leftBack,rowboat);

        going_to_pt = false;


        //Move collector_arm up
        //collector_arm.setPosition(0.77);







        //foundation_mover.setPosition(0);

        //WebcamName camName = hardwareMap.get(WebcamName.class,"Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));


         //Open the connection to the camera device

        webcam.openCameraDevice();


//        Specify the image processing pipeline we wish to invoke upon receipt
//        of a frame from the camera. Note that switching pipelines on-the-fly
//        (while a streaming session is in flight) *IS* supported.


        pip = new StonePipeline(800,0,320,240,60);
        webcam.setPipeline(pip);

//
//        Tell the webcam to start streaming images to us! Note that you must make sure
//        the resolution you specify is supported by the camera. If it is not, an exception
//        will be thrown.
//
//        Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
//        supports streaming from the webcam in the uncompressed YUV image format. This means
//        that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
//        Streaming at 720p will limit you to up to 10FPS. However, streaming at frame rates other
//        than 30FPS is not currently supported, although this will likely be addressed in a future
//        release. TLDR: You can't stream in greater than 480p from a webcam at the moment.
//
//        Also, we specify the rotation that the webcam is used in. This is so that the image
//        from the camera sensor can be rotated such that it is always displayed with the image upright.
//        For a front facing camera, rotation is defined assuming the user is looking at the screen.
//        For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
//        away from the user.
//
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        vertCont = new MotorController(vertical_extender,new PIDController(0.002,0.0001,0.002,0));
    }


    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

    @Override
    public void init_loop() {

    }


    //Code to run ONCE when the driver hits PLAY

    @Override
    public void start() {
        int stonePos = 2-pip.stonePos;
        webcam.stopStreaming();
        getStone(stonePos+3,700,(Object stone)->{
            control.gotoPoint(new Transform(-2254,770,-Math.PI*0.5),true,0.35,0.85,60,(Object alphabet)->{
                left_stone_collector_arm.setPosition(0.74);
                try {
                    Thread.sleep(273);
                    left_stone_collector.setPosition(0.01);
                    Thread.sleep(284);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                control.gotoPoint(new Transform(-2254,550,-Math.PI*0.5),true
                        ,0.25,0.5,80,(Object abbcdea)->{
                            left_stone_collector_arm.setPosition(1);
                            getStone(stonePos,676,(Object stone1)->{
                        control.gotoPoint(new Transform(-2021,740,-Math.PI*0.5),true,0.35,1,60,(Object alphabetcdefg)->{
                            left_stone_collector_arm.setPosition(0.74);
                            try {
                                Thread.sleep(273);
                                left_stone_collector.setPosition(0.01);
                                Thread.sleep(284);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            left_stone_collector_arm.setPosition(1);
                            control.gotoPoint(new Transform(-2021,520,-Math.PI*0.5),true,0.25,0.75,40,(Object abcdefhlep)->{
                                control.gotoPoint(new Transform(-1000,520,-Math.PI*0.5),true,0.25,1,40,(Object icnEngilrsh)->0);
                                return 0;
                            });
                            return 0;

                        });
                        return 0;
                    });
                    return 0;
                });
                return 0;
            });
            return 0;
        });
        Interval delay = new Interval((Object obj42)->{

            try {
                Thread.sleep(250);
                collector_arm.setPosition(0.63);
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            vertCont.setTarget(1302,true);
            return 1;
        },0);
        delay.start();
        runtime.reset();
    }


    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    @Override
    public void loop()
    {
        rowboat.relocalize();
        vertCont.updateController();
        telemetry.addData("color",pip.stonePos);
    }


//    Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {
    }

    private void getStone(int stoneNum,double y, Lambda callback)
    {
        control.gotoPoint(new Transform(Math.min(-322+(200*stoneNum),558),y-150,-Math.PI*0.5),true,0.35,0.8,30,(Object obj)->{
            left_stone_collector.setPosition(0.43);
            left_stone_collector_arm.setPosition(0.74);
            control.gotoPoint(new Transform(-322+(200*stoneNum),y,-Math.PI*0.5),true,0.35,0.5,20,(Object obj1)->{
                try {
                    left_stone_collector_arm.setPosition(0.69);
                    Thread.sleep(200);
                    left_stone_collector.setPosition(0.56);
                    Thread.sleep(500);
                    left_stone_collector_arm.setPosition(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                control.gotoPoint(new Transform(-322+(200*stoneNum),y-150,-Math.PI*0.5),true,35,0.5,80,(Object abbcdea)->{
                    callback.call(stoneNum);
                    return 0;
                });
                return 0;
            });
            return 0;
        });
    }

}
*/

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.MotorController;
import org.firstinspires.ftc.teamcode.Robot.PIDController;
import org.firstinspires.ftc.teamcode.Robot.Robot_Controller;
import org.firstinspires.ftc.teamcode.Robot.Robot_Localizer;
import org.firstinspires.ftc.teamcode.Robot.StonePipeline;
import org.firstinspires.ftc.teamcode.Utils.Interval;
import org.firstinspires.ftc.teamcode.Utils.Lambda;
import org.firstinspires.ftc.teamcode.Utils.Transform;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
//@Disabled
public class FullRedAuto extends OpMode {
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
    private Servo right_stone_collector;
    private Servo right_stone_collector_arm;
    private Servo left_stone_collector;
    private Servo left_stone_collector_arm;

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

    private MotorController vertCont;


    @Override
    public void init() {
        leftFront           = hardwareMap.get(DcMotor.class, "left_front");
        rightFront          = hardwareMap.get(DcMotor.class, "right_front");
        leftBack            = hardwareMap.get(DcMotor.class, "left_back");
        rightBack           = hardwareMap.get(DcMotor.class, "right_back");
        horizontal_extender = hardwareMap.get(DcMotor.class, "horizontal_ext");
        vertical_extender   = hardwareMap.get(DcMotor.class, "vertical_ext");

        collector_arm       = hardwareMap.get(Servo.class, "collector_arm");
        foundation_mover    = hardwareMap.get(Servo.class, "Foundation_mover");
        right_stone_collector = hardwareMap.get(Servo.class, "right_stone_collector");
        right_stone_collector_arm = hardwareMap.get(Servo.class, "right_stone_collector_arm");
        left_stone_collector = hardwareMap.get(Servo.class, "left_stone_collector");
        left_stone_collector_arm = hardwareMap.get(Servo.class, "left_stone_collector_arm");

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
        horizontal_extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vertical_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical_extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rowboat = new Robot_Localizer(leftBack,rightFront,rightBack,0.958);
        control = new Robot_Controller(rightFront,leftFront,rightBack,leftBack,rowboat,true);

        going_to_pt = false;


        //Move collector_arm up
        //collector_arm.setPosition(0.77);







        //foundation_mover.setPosition(0);

        //WebcamName camName = hardwareMap.get(WebcamName.class,"Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Open the connection to the camera device
         */
        webcam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */

        pip = new StonePipeline(800,0,320,240,60);
        webcam.setPipeline(pip);

        /*
         * Tell the webcam to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
         * supports streaming from the webcam in the uncompressed YUV image format. This means
         * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
         * Streaming at 720p will limit you to up to 10FPS. However, streaming at frame rates other
         * than 30FPS is not currently supported, although this will likely be addressed in a future
         * release. TLDR: You can't stream in greater than 480p from a webcam at the moment.
         *
         * Also, we specify the rotation that the webcam is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        vertCont = new MotorController(vertical_extender,new PIDController(0.002,0.0001,0.002,0));
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
        int stonePos = 2-pip.stonePos;
        webcam.stopStreaming();
        getStone(stonePos+3,700,175,(Object stone)->{
            control.gotoPoint(new Transform(2254,770,Math.PI*0.5),true,0.35,0.85,100,(Object alphabet)->{
                left_stone_collector_arm.setPosition(0.84);
                left_stone_collector.setPosition(0.01);
                try {
                    Thread.sleep(284);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                control.gotoPoint(new Transform(2254,550,Math.PI*0.5),true
                        ,0.25,0.5,80,(Object abbcdea)->{
                            left_stone_collector_arm.setPosition(1);
                            getStone(stonePos,650,200,(Object stone1)->{
                                control.gotoPoint(new Transform(2021,760,Math.PI*0.5),true,0.35,1,100,(Object alphabetcdefg)->{
                                    left_stone_collector_arm.setPosition(0.84);
                                    try {
                                        Thread.sleep(273);
                                        left_stone_collector.setPosition(0.01);
                                        Thread.sleep(284);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    left_stone_collector_arm.setPosition(1);
                                    control.gotoPoint(new Transform(2021,590,Math.PI*0.5),true,0.25,0.85,50,(Object afjrj)->{
                                        control.gotoPoint(new Transform(1000,590,Math.PI*0.5),true,0.5,0.85,150,(Object abcdefhlep)->0);
                                        return 0;
                                    });
                                    return 0;
                                });
                                return 0;
                            });
                            return 0;
                        });
                return 0;
            });
            return 0;
        });
        Interval delay = new Interval((Object obj42)->{

            try {
                Thread.sleep(250);
                collector_arm.setPosition(0.63);
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            vertCont.setTarget(1302,true);
            return 1;
        },0);
        delay.start();
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        rowboat.relocalize();
        vertCont.updateController();
        telemetry.addData("color",pip.stonePos);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        webcam.stopStreaming();
    }

    private void getStone(int stoneNum,double y,double bDist, Lambda callback)
    {
        control.gotoPoint(new Transform(Math.max(350-(200*(stoneNum)),-558),y-150,Math.PI*0.5),true,0.25,0.8,60,-Math.PI*0.5,(Object obj)->{
            left_stone_collector.setPosition(0.43);
            left_stone_collector_arm.setPosition(0.74);
            control.gotoPoint(new Transform(350-(200*(stoneNum)),y,Math.PI*0.5),true,0.35,0.5,35,(Object obj1)->{
                try {
                    left_stone_collector_arm.setPosition(0.69);
                    Thread.sleep(200);
                    left_stone_collector.setPosition(0.56);
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                left_stone_collector_arm.setPosition(1);
                control.gotoPoint(new Transform(350-(200*(stoneNum)),y-bDist,Math.PI*0.5),true,0.35,0.5,80,(Object abbcdea)->{
                    callback.call(stoneNum);
                    return 0;
                });
                return 0;
            });
            return 0;
        });
    }

}
