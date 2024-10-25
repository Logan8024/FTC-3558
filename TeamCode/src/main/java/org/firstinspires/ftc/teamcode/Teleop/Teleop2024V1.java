package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Teleop 2024 V1", group="Linear OpMode")

public class Teleop2024V1 extends LinearOpMode{

    //declare motor values
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
    double BeltPower = 1;
    double WinchPower = .95;
    double WinchAutoPower = .5;
    double ArmPower = .25;
    int[] ArmPositions = new int[]{0,130,90,55};
    //counts per rotation 1,527.793876
    boolean Climbing = false;

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

        //Gyro
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters IMUparameters;
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.initialize(parameters);

        //arm positions converted from degree to counts per rotation calculated

        for (int i = 0; i<4; i++) {
            ArmPositions[i] = (int)Math.round((1527.793876 / 360) * ArmPositions[i]);
        }

        //Variables for drive

        double LeftPower;
        double RightPower;
        double LeftStrafePower;
        double RightStrafePower;

        //Wait For Game To Start

        waitForStart();

        //Set Default Motor Positions

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        Winch.setDirection(DcMotor.Direction.REVERSE);
        Arm.setDirection(DcMotor.Direction.REVERSE);
        ClawServo.setPosition(0);
        BeltServo.setPower(0);

        //Arm Encoder Values

        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(ArmPower);

        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

            //Joy Stick Values for Driving

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;

            //Field Centric Controls
            if (this.gamepad1.right_bumper) {
                LeftPower	= Range.clip(drive + turn, -1.0, 1.0) *.75;
                RightPower   = Range.clip(drive - turn, -1.0, 1.0) *.75;
                LeftStrafePower = strafe*.75;
                RightStrafePower = strafe*.75;



                // Send calculated power to wheels
                FrontLeft.setPower(LeftPower);
                FrontRight.setPower(RightPower);
                BackLeft.setPower(LeftPower);
                BackRight.setPower(RightPower);
                if(Math.abs(strafe) > 0.1){
                    FrontLeft.setPower(LeftStrafePower);
                    FrontRight.setPower(-RightStrafePower);
                    BackLeft.setPower(-LeftStrafePower);
                    BackRight.setPower(RightStrafePower);
                }
            }
            else {
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
                double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

                rotX = rotX * 1.1;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(strafe), 1);
                double frontLeftPower = (rotY + rotX + turn) / denominator;
                double backLeftPower = (rotY - rotX + turn) / denominator;
                double frontRightPower = (rotY - rotX - turn) / denominator;
                double backRightPower = (rotY + rotX - turn) / denominator;

                FrontLeft.setPower(frontLeftPower);
                BackLeft.setPower(backLeftPower);
                BackRight.setPower(backRightPower);
                FrontRight.setPower(frontRightPower);
            }
            //Game Controls

            //Gyroscope

            if (this.gamepad1.right_trigger == 1) {
                imu.resetYaw();
                telemetry.addData("Gyroscope", "Reset");
                telemetry.update();

            }

            //Toggle Button for Claw
            if (this.gamepad1.left_bumper) {
                //Close
                ClawServo.setPosition(0.62);
            }
            if (this.gamepad1.left_trigger == 1) {
                //Open
                ClawServo.setPosition(1);
            }

            //Control for Belt Servo
            if (this.gamepad2.left_trigger == 1) {
                BeltServo.setPower(BeltPower);
            }
            else if (this.gamepad2.left_bumper) {
                BeltServo.setPower(-BeltPower);
            }
            else {
                BeltServo.setPower(0);
            }

            //Winch

            if (this.gamepad2.right_trigger == 1) {
                Winch.setDirection(DcMotor.Direction.FORWARD);
                Winch.setPower(WinchPower);
            }
            else if (this.gamepad2.right_bumper) {
                Winch.setDirection(DcMotor.Direction.REVERSE);
                Winch.setPower(WinchPower);
            }
            else {
                Winch.setPower(0);
            }

            //Arm

            if (this.gamepad1.y) {
                Arm.setPower(.5);
                Arm.setTargetPosition(ArmPositions[0]);
            }
            else if (this.gamepad1.b) {
                Arm.setPower(.5);
                Arm.setTargetPosition(ArmPositions[1]);
            }
            else if (this.gamepad1.a) {
                Arm.setPower(.5);
                Arm.setTargetPosition(ArmPositions[2]);
            }
            else if (this.gamepad1.x) {
                Arm.setPower(.85);
                Arm.setTargetPosition(ArmPositions[3]);
            }

            //Climb Auto by Button

            /*if (this.gamepad2.start) {
                Climbing = true;
                runtime.reset();
                while (Climbing) {
                    if (runtime.seconds() > 0) {
                        Winch.setDirection(DcMotor.Direction.FORWARD);
                        Winch.setPower(WinchAutoPower);
                        if (runtime.seconds() > 2) {
                            BeltServo.setPower(BeltPower);
                            if (runtime.seconds() > 6) {
                                Winch.setPower(0);
                                BeltServo.setPower(0);
                                Climbing = false;
                                runtime.reset();
                            }
                        }
                    }
                }
            }*/
        }
    }
}


