package org.firstinspires.ftc.teamcode.Teleop.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Teleop.Constants;

public class Claw {
    private static Servo clawServo = Constants.MotorDefinitions.ClawServo;
    private static Servo clawRotation = Constants.MotorDefinitions.ClawRotation;
    public static void Open() {
        clawServo.setPosition(0);
    }
    public static void Close() {
        clawServo.setPosition(.8);
    }
    public static void Rotate() {
        clawRotation.setPosition(1);
    }
    public static void Straight() {
        clawRotation.setPosition(.30);
    }
}

