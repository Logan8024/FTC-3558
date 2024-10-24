package org.firstinspires.ftc.teamcode.Auto.SpecimenAutos;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RightPark")
public class RightPark extends LinearOpMode {
     DcMotor FrontLeft = null;
     DcMotor FrontRight = null;
     DcMotor BackLeft = null;
     DcMotor BackRight = null;
     Servo ClawServo = null;
     CRServo BeltServo = null;
     DcMotor Winch;
     DcMotor Arm;
     IMU imu;
     ElapsedTime	 runtime = new ElapsedTime();


    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Hardware Maps
        FrontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        FrontRight = hardwareMap.get(DcMotor.class, "rightFront");
        BackRight = hardwareMap.get(DcMotor.class, "rightBack");
        BackLeft = hardwareMap.get(DcMotor.class, "leftBack");
        Winch = hardwareMap.get(DcMotor.class, "Winch");
        Arm = hardwareMap.get(DcMotorEx.class, "Arm");
        ClawServo = hardwareMap.get(Servo.class, "ArmServo");
        BeltServo = hardwareMap.get(CRServo.class, "BeltServo");

        waitForStart();

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        Winch.setDirection(DcMotor.Direction.REVERSE);
        Arm.setDirection(DcMotor.Direction.FORWARD);

        //Arm Encoder Values

        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ClawServo.setPosition(0.62);

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() > 0 && runtime.seconds() < 2.5) {
            FrontRight.setPower(-.5);
            FrontLeft.setPower(.5);
            BackRight.setPower(.5);
            BackLeft.setPower(-.5);
        }
        while (opModeIsActive() && runtime.seconds() > 2.5) {
            FrontRight.setPower(0);
            FrontLeft.setPower(0);
            BackRight.setPower(0);
            BackLeft.setPower(0);
        }
    }
}

