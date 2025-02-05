package org.firstinspires.ftc.teamcode.Teleop.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Teleop.Constants;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.JoyStick.JoyStick1;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive {
    static double LeftPower;
    static double RightPower;
    static double LeftStrafePower;
    static double RightStrafePower;

    static DcMotor FrontLeft = Constants.MotorDefinitions.FrontLeft;
    static DcMotor FrontRight = Constants.MotorDefinitions.FrontRight;
    static DcMotor BackLeft = Constants.MotorDefinitions.BackLeft;
    static DcMotor BackRight = Constants.MotorDefinitions.BackRight;
    static IMU imu = Constants.MotorDefinitions.imu;

    static double drive = JoyStick1.LeftStick.x;
    static double turn = JoyStick1.LeftStick.y;
    static double strafe = JoyStick1.RightStick.x;

    public static void RobotCentric() {
        LeftPower = Range.clip(drive + turn, -1.0, 1.0) * .75;
        RightPower = Range.clip(drive - turn, -1.0, 1.0) * .75;
        LeftStrafePower = strafe * .75;
        RightStrafePower = strafe * .75;


        // Send calculated power to wheels
        FrontLeft.setPower(LeftPower);
        FrontRight.setPower(RightPower);
        BackLeft.setPower(LeftPower);
        BackRight.setPower(RightPower);

        if (Math.abs(strafe) > 0.1) {
            FrontLeft.setPower(LeftStrafePower);
            FrontRight.setPower(-RightStrafePower);
            BackLeft.setPower(-LeftStrafePower);
            BackRight.setPower(RightStrafePower);
        }
    }

    public static void FieldCentric() {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(strafe), 1);
        double frontLeftPower = (rotY + rotX + turn) / denominator;
        double backLeftPower = (rotY - rotX + turn) / denominator;
        double frontRightPower = (rotY - rotX - turn) / denominator;
        double backRightPower = (rotY + rotX - turn) / denominator;

        FrontLeft.setPower(frontLeftPower);
        BackLeft.setPower(backLeftPower);
        BackRight.setPower(backRightPower);
        FrontRight.setPower(frontRightPower);
    }
}
