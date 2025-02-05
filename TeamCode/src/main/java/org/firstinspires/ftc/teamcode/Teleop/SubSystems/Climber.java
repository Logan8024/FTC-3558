package org.firstinspires.ftc.teamcode.Teleop.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Teleop.Constants;

public class Climber {
    public static DcMotor Climber = Constants.MotorDefinitions.Winch;
    public static void Down() {
        Climber.setPower(-1);
    }
    public static void Up() {
        Climber.setPower(1);
    }
}
