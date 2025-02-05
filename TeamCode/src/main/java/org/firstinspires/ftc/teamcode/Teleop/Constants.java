package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Constants {
    public static class Power {
        public static double BeltPower = 1;
        public static double WinchPower = .95;
        public static double ArmPower = .25;
    }

    public class ArmPositions {
        int[] ArmPositions = new int[]{0,130,90,55,110,65};
    }

    public static class MotorDefinitions {
        public static DcMotor FrontLeft = hardwareMap.get(DcMotor .class, "leftFront");
        public static DcMotor FrontRight = hardwareMap.get(DcMotor.class, "rightFront");
        public static DcMotor BackRight = hardwareMap.get(DcMotor.class, "rightBack");
        public static DcMotor BackLeft = hardwareMap.get(DcMotor.class, "leftBack");
        public static DcMotor Winch = hardwareMap.get(DcMotor.class, "Winch");
        public static DcMotor WinchHook = hardwareMap.get(DcMotor.class, "WinchHook");
        public static DcMotorEx Arm = hardwareMap.get(DcMotorEx .class, "Arm");
        public static Servo ClawServo = hardwareMap.get(Servo .class, "ClawServo");
        public static Servo ClawRotation = hardwareMap.get(Servo.class, "ClawRotation");
        public static CRServo BeltServo = hardwareMap.get(CRServo .class, "BeltServo");
        public static Limelight3A Limelight = hardwareMap.get(Limelight3A .class, "limelight");
        public static IMU imu = hardwareMap.get(IMU .class, "imu");
    }


}
