package org.firstinspires.ftc.teamcode.Auto.Config;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AutonomousCommands{

    static ElapsedTime runtime = new ElapsedTime();
    public static int[] ArmPositions = new int[]{0,165,200,280};
    public static int[] ArmPositionsAuto = new int[]{0,382,212};


    public static void Forward(double Time, double Speed,DcMotor[] Motors) {
        runtime.reset();
        while (runtime.seconds() < Time) {
            Motors[0].setPower(Speed);
            Motors[1].setPower(Speed);
            Motors[2].setPower(Speed);
            Motors[2].setPower(Speed);
        }
    }

    public static void Backward(double Time, double Speed,DcMotor[] Motors) {
        runtime.reset();
        while (runtime.seconds() < Time) {
            Motors[0].setPower(-Speed);
            Motors[1].setPower(-Speed);
            Motors[2].setPower(-Speed);
            Motors[2].setPower(-Speed);
        }
    }

    public static void Turn(double Time,double Speed, DcMotor[] Motors) {
        runtime.reset();
        while (runtime.seconds() < Time) {
            Motors[0].setPower(Speed);
            Motors[1].setPower(-Speed);
            Motors[2].setPower(-Speed);
            Motors[3].setPower(Speed);
        }
    }

    public static void Strafe(double Time, double Speed, DcMotor[] Motors) {
        runtime.reset();
        while (runtime.seconds() < Time) {
            Motors[0].setPower(Speed);
            Motors[1].setPower(-Speed);
            Motors[2].setPower(Speed);
            Motors[3].setPower(-Speed);
        }
    }
    public static void Stop(DcMotor[] Motors) {
        Motors[0].setPower(0);
        Motors[1].setPower(0);
        Motors[2].setPower(0);
        Motors[3].setPower(0);
    }


    public static void MoveArm(int i,double Time, double Power, DcMotor Arm) {
        runtime.reset();
        while (runtime.seconds() < Time) {
            Arm.setPower(Power);
            Arm.setTargetPosition(ArmPositionsAuto[i]);
        }
    }
    public static void Wait(double Time) {
        runtime.reset();
        while (runtime.seconds() < Time) {
            //Do Nothing
        }
    }

    public static void Claw(double Position, Servo ClawServo) {
        ClawServo.setPosition(Position);
    }
}
