package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.JoyStick;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.JoyStick.JoyStick1;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.JoyStick.JoyStick2;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Drive;
import org.firstinspires.ftc.teamcode.Teleop.SubSystems.Platform;


@TeleOp(name = "TeleOp2024V2")
public class TeleOp2024V2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                telemetry.addData("Controller1", JoyStick1.ID());
                telemetry.addData("Controller2", JoyStick2.ID());
                //Drive Mode
                JoyStick.Command(JoyStick1.RightBumper, true, Drive::RobotCentric);
                JoyStick.Command(JoyStick1.RightBumper, false, Drive::FieldCentric);

                //Belt Movement
                JoyStick.Command(JoyStick1.LeftTrigger, true, Platform::Forward);
                JoyStick.Command(JoyStick1.LeftBumper, true, Platform::Backward);

                //Claw Movement
                JoyStick.Command(JoyStick1.RightTrigger, true, Claw::Open);
                JoyStick.Command(JoyStick1.RightBumper, true, Claw::Close);
                JoyStick.Command(JoyStick1.DpadUp, true, Claw::Straight);
                JoyStick.Command(JoyStick1.DpadLeft, true, Claw::Rotate);

                //Climber Movement

            }
        }
    }
}
