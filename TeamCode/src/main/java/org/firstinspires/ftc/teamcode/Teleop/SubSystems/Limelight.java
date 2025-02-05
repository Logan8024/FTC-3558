package org.firstinspires.ftc.teamcode.Teleop.SubSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Teleop.Constants.MotorDefinitions;

public class Limelight {
    public double Get_Angle () {

        Limelight3A Limelight = MotorDefinitions.Limelight;
        LLResult Result = Limelight.getLatestResult();

        double Angle;

        if  (Result != null) {
            double[] pythonOutputs = Result.getPythonOutput();
            if (pythonOutputs != null && pythonOutputs.length > 0) {
                Angle = (((1/40500) * (pythonOutputs[0] * pythonOutputs[0])) - (.01 * pythonOutputs[0]) + 1);
            } else {
                Angle = 0; // Or some other default to indicate no target detected
            }
            return Angle;
        }
        return 0;
    }
}
