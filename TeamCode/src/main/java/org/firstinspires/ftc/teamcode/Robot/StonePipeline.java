package org.firstinspires.ftc.teamcode.Robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class StonePipeline extends OpenCvPipeline
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
    double stoneBase;

    public int stonePos = 0;

    public StonePipeline(double camDist, double camX, int screenWidth,int screenHeight, int stoneTop)
    {
       stoneScreenSize =( 200/camDist)*screenWidth;
       this.stoneTop = stoneTop;
       this.stoneBase = stoneTop+(100/camDist)*screenHeight;
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


        Scalar s1Color;
        Scalar s2Color;
        Scalar s3Color;
        if(stonePos == 0)
        {
            s1Color = new Scalar(255,0,0);
            s2Color = new Scalar(0,255,0);
            s3Color = new Scalar(0,255,0);
        }
        else if(stonePos == 1)
        {
            s1Color = new Scalar(0,255,0);
            s2Color = new Scalar(255,0,0);
            s3Color = new Scalar(0,255,0);
        }
        else
        {
            s1Color = new Scalar(0,255,0);
            s2Color = new Scalar(0,255,0);
            s3Color = new Scalar(255,0,0);
        }
        Imgproc.rectangle(
                input,
                new Point(
                        stoneScreenSize*0.5,
                        stoneTop),
                new Point(
                        stoneScreenSize*1.5,
                        stoneBase),
                s1Color, 4);
        Imgproc.rectangle(
                input,
                new Point(
                        stoneScreenSize*1.5,
                        stoneTop),
                new Point(
                        stoneScreenSize*2.5,
                        stoneBase),
                s2Color, 4);
        Imgproc.rectangle(
                input,
                new Point(
                        stoneScreenSize*2.5,
                        stoneTop),
                new Point(
                        stoneScreenSize*3.5,
                        stoneBase),
                s3Color, 4);

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
    private Scalar mean(Mat input, int hopDist, int top, double bottom, int left, int right)
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

