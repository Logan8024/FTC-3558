package org.firstinspires.ftc.teamcode.Auto.SpecimenAutos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auto.Config.AutonomousCommands;

@Autonomous(name = "AutoCommandTest")
public class AutoCommandTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Gyro
        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters IMUparameters;
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.initialize(parameters);

        //Motor Declaration
        DcMotor[] DriveMotors = new DcMotor[]{
                hardwareMap.get(DcMotor.class, "leftFront"),
                hardwareMap.get(DcMotor.class, "rightFront"),
                hardwareMap.get(DcMotor.class, "rightBack"),
                hardwareMap.get(DcMotor.class, "leftBack")
        };

        Servo ClawServo = hardwareMap.get(Servo.class, "ArmServo");

        DcMotor Arm = hardwareMap.get(DcMotorEx.class, "Arm");
        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setDirection(DcMotor.Direction.REVERSE);
        Arm.setPower(0.25);

        waitForStart();
        DriveMotors[0].setDirection(DcMotor.Direction.REVERSE);
        DriveMotors[1].setDirection(DcMotor.Direction.FORWARD);
        DriveMotors[2].setDirection(DcMotor.Direction.FORWARD);
        DriveMotors[3].setDirection(DcMotor.Direction.FORWARD);

        while (opModeIsActive()) {
            AutonomousCommands.Stop(DriveMotors);
            AutonomousCommands.Claw(.5,ClawServo);
            AutonomousCommands.MoveArm(1,.5,.25,Arm);
            AutonomousCommands.Forward(1.75, .3, DriveMotors);
            AutonomousCommands.Stop(DriveMotors);
            AutonomousCommands.MoveArm(2,.75,.95,Arm);
            AutonomousCommands.Claw(1,ClawServo);
            AutonomousCommands.Backward(.25, .4, DriveMotors);
            AutonomousCommands.Turn(2, .4, DriveMotors);
            AutonomousCommands.Strafe(2.4, -.35, DriveMotors);
            AutonomousCommands.Stop(DriveMotors);
            AutonomousCommands.MoveArm(0,5.0,.2,Arm);
            AutonomousCommands.Forward(.8 , .1, DriveMotors);
            AutonomousCommands.Stop(DriveMotors);
            AutonomousCommands.Claw(.5,ClawServo);
            AutonomousCommands.Wait(.4);
            AutonomousCommands.Forward(.8, -.1, DriveMotors);
            AutonomousCommands.MoveArm(1,.4,.25,Arm);
            AutonomousCommands.Turn(2, -.4, DriveMotors);
            AutonomousCommands.Strafe(2.4, -.35, DriveMotors);
            AutonomousCommands.Forward(.3, .4,DriveMotors);
            AutonomousCommands.Stop(DriveMotors);
            AutonomousCommands.MoveArm(2,.4,.95,Arm);
            AutonomousCommands.Claw(1,ClawServo);
            AutonomousCommands.Backward(1.2, .4, DriveMotors);
            AutonomousCommands.Strafe(2.4, -.4, DriveMotors);
            AutonomousCommands.Stop(DriveMotors);
            break;
        }



    }
}
