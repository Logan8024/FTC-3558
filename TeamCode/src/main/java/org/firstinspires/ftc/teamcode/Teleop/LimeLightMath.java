package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;


public class LimeLightMath extends LinearOpMode{

    public LLResultTypes.ColorResult Result;
    public Limelight3A Limelight;
    public List<List<Double>> CornerList;
    public List<Double> Hypotenuse;
    public List<Double> SideX;
    public List<Double> SideY;
    public double SmallX;
    public double SmallY;
    public int SmallestSide;
    public double Rotation;
    public void runOpMode() {

        //Limelight
        Limelight = hardwareMap.get(Limelight3A.class, "limelight");

        Limelight.pipelineSwitch(2);

        Limelight.start();



        while (opModeIsActive()) {
            CornerList = Result.getTargetCorners();

            //Limelight Math
            for (int i = 0; i<4; i++) {
                    double X1;
                    double Y1;
                    double X2;
                    double Y2;
                if (i == 3) {
                    X1 = CornerList.get(i).get(0);
                    Y1 = CornerList.get(i).get(1);
                    X2 = CornerList.get(0).get(0);
                    Y2 = CornerList.get(0).get(1);
                }
                else {
                    X1 = CornerList.get(i).get(0);
                    Y1 = CornerList.get(i).get(1);
                    X2 = CornerList.get(i + 1).get(0);
                    Y2 = CornerList.get(i + 1).get(1);
                }
                Double Hyp = Math.hypot((X2-X1), (Y2 - Y1));

                Hypotenuse.set(i, Hyp);
                SideX.set(i, (X2-X1));
                SideY.set(i, (Y2-Y1));
            }
            for (int i = 0; i < 4; i++) {
                if (i == 3) {
                    if (Hypotenuse.get(i) > Hypotenuse.get(0)) {
                        SmallestSide = i+1;
                    }
                    else {
                        SmallestSide = i;
                    }
                }
                else {
                    if (Hypotenuse.get(i) > Hypotenuse.get(i+1)) {
                        SmallestSide = i+1;
                    }
                    else
                        SmallestSide = i;
                    }
                }
            }

        Rotation = Math.atan2(SideY.get(SmallestSide), SideX.get(SmallestSide));










        }
    }
