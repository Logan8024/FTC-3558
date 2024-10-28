package org.firstinspires.ftc.teamcode.Auto.Config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


//Change to File Name
//@Autonomous(name = "AutonomousTemplate")
//Change to File Name
public class AutonomousTemplate extends LinearOpMode {
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
            //Put Auto Commands Here

            break;
        }



    }
}
