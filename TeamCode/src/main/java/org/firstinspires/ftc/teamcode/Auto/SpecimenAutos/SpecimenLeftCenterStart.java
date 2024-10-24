package org.firstinspires.ftc.teamcode.Auto.SpecimenAutos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "SpecimenLeftCenterStart")
@Config
public class SpecimenLeftCenterStart extends LinearOpMode {
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

    //declare determinant variables
     double BeltPower = 5;
     double WinchPower = .95;
     double WinchAutoPower = .5;
     double ArmPower = .25;
     int[] ArmPositions = new int[]{0,165,200,280};
     int[] ArmPositionsAuto = new int[]{0,90,50};

    //counts per rotation 1,527.793876

    //drive
     static double TurnLength = 2.6;
     static double TurnSpeed= 0.3;
     static double StrafeLength = 2.35;
     static double StrafeSpeed = -.35;
     static double ForwardSpeed = .1;
     static double ForwardLength = .6;


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

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        Winch.setDirection(DcMotor.Direction.REVERSE);
        Arm.setDirection(DcMotor.Direction.REVERSE);

        //Arm Encoder Values

        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(ArmPower);

        //Auto
        for (int i = 0; i<4; i++) {
            ArmPositions[i] = (int)Math.round((1527.793876 / 360) * ArmPositions[i]);
        }
        for (int i = 0; i<3; i++) {
            ArmPositionsAuto[i] = (int)Math.round((1527.793876 / 360) * ArmPositionsAuto[i]);
        }
        ClawServo.setPosition(0.5);
        runtime.reset();

        //Move Forward
        while (opModeIsActive() && runtime.seconds() < 1.2) {
            Arm.setTargetPosition(ArmPositionsAuto[1]);
            ClawServo.setPosition(.5);
            FrontRight.setPower(.4);
            FrontLeft.setPower(.4);
            BackRight.setPower(.4);
            BackLeft.setPower(.4);
        }
        runtime.reset();
        //stop
        while (opModeIsActive() && runtime.seconds() < .2) {
            FrontRight.setPower(0);
            FrontLeft.setPower(0);
            BackRight.setPower(0);
            BackLeft.setPower(0);
        }
        runtime.reset();
        //Move Arm Down To Clip
        while (opModeIsActive() && runtime.seconds() < .24) {
            Arm.setPower(.75);
            Arm.setTargetPosition(ArmPositionsAuto[2]);
        }
        runtime.reset();
        //Release Claw and Move Backwards
        while (opModeIsActive() && runtime.seconds() < .3){
            ClawServo.setPosition(1);
            FrontRight.setPower(-.4);
            FrontLeft.setPower(-.4);
            BackRight.setPower(-.4);
            BackLeft.setPower(-.4);
        }
        runtime.reset();
        //Rotate 180
        while (opModeIsActive() && runtime.seconds() < TurnLength) {
            FrontRight.setPower(-TurnSpeed);
            FrontLeft.setPower(TurnSpeed);
            BackRight.setPower(-TurnSpeed);
            BackLeft.setPower(TurnSpeed);
        }
        runtime.reset();
        //Strafe Right
        while (opModeIsActive() && runtime.seconds() < StrafeLength) {
            FrontRight.setPower(-StrafeSpeed);
            FrontLeft.setPower(StrafeSpeed);
            BackRight.setPower(StrafeSpeed);
            BackLeft.setPower(-StrafeSpeed);
        }
        runtime.reset();
        //Move Arm Down to Pickup and Move forward
        while (opModeIsActive() && runtime.seconds() < 5) {
            Arm.setPower(.2);
            Arm.setTargetPosition(ArmPositionsAuto[0]);
            FrontRight.setPower(0);
            FrontLeft.setPower(0);
            BackRight.setPower(0);
            BackLeft.setPower(0);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < ForwardLength) {
            FrontRight.setPower(ForwardSpeed);
            FrontLeft.setPower(ForwardSpeed);
            BackRight.setPower(ForwardSpeed);
            BackLeft.setPower(ForwardSpeed);
        }
        runtime.reset();
        //Stop
        while (opModeIsActive() && runtime.seconds() < .5) {
            FrontRight.setPower(0);
            FrontLeft.setPower(0);
            BackRight.setPower(0);
            BackLeft.setPower(0);
        }
        //Close Claw
        while (opModeIsActive() && runtime.seconds() < .2) {
            ClawServo.setPosition(0.5);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < ForwardLength) {
            Arm.setPower(.25);
            Arm.setTargetPosition(ArmPositionsAuto[1]);
            FrontRight.setPower(-ForwardSpeed);
            FrontLeft.setPower(-ForwardSpeed);
            BackRight.setPower(-ForwardSpeed);
            BackLeft.setPower(-ForwardSpeed);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < TurnLength+.2) {
            FrontRight.setPower(TurnSpeed);
            FrontLeft.setPower(-TurnSpeed);
            BackRight.setPower(TurnSpeed);
            BackLeft.setPower(-TurnSpeed);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < StrafeLength - .3) {
            FrontRight.setPower(-StrafeSpeed);
            FrontLeft.setPower(StrafeSpeed);
            BackRight.setPower(StrafeSpeed);
            BackLeft.setPower(-StrafeSpeed);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .65){
            FrontRight.setPower(.4);
            FrontLeft.setPower(.4);
            BackRight.setPower(.4);
            BackLeft.setPower(.4);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .4) {
            FrontRight.setPower(0);
            FrontLeft.setPower(0);
            BackRight.setPower(0);
            BackLeft.setPower(0);
            Arm.setPower(.75);
            Arm.setTargetPosition(ArmPositionsAuto[2]);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .36) {
            ClawServo.setPosition(1);
        }





    }
}

