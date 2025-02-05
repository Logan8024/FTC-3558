package org.firstinspires.ftc.teamcode.Teleop.SubSystems;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Teleop.Constants;

public class Platform {
    static CRServo belt = Constants.MotorDefinitions.BeltServo;
    public static void Forward() {
        belt.setPower(1);
    }
    public static void Backward() {
        belt.setPower(-1);
    }
}
