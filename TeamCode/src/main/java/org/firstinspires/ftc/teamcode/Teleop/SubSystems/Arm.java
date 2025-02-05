package org.firstinspires.ftc.teamcode.Teleop.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Teleop.Constants;

public class Arm {
    public static DcMotorEx arm = Constants.MotorDefinitions.Arm;
    public static double ArmPower = Constants.Power.ArmPower;
    public void Configure() {
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(ArmPower);
    }

}
