package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="Teleop 2024 V1", group="Linear OpMode")
public class Teleop2024V1 extends LinearOpMode{

    //Motors

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    Servo ClawServo;
    Servo ClawRotation;
    CRServo BeltServo;
    DcMotor Winch;
    DcMotor WinchHook;
    DcMotor Arm;

    //Data

    IMU imu;
    Limelight3A Limelight;

    //Time

    ElapsedTime	 runtime = new ElapsedTime();

    //determinant variables

    double BeltPower = 1;
    double WinchPower = .95;
    double ArmPower = .25;
    int[] ArmPositions = new int[]{0,130,90,55,110,150};
    //counts per rotation 1,527.793876

    //Limelight Math Variables
    double Angle;


    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //Hardware Maps

        FrontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        FrontRight = hardwareMap.get(DcMotor.class, "rightFront");
        BackRight = hardwareMap.get(DcMotor.class, "rightBack");
        BackLeft = hardwareMap.get(DcMotor.class, "leftBack");
        Winch = hardwareMap.get(DcMotor.class, "Winch");
        WinchHook = hardwareMap.get(DcMotor.class, "WinchHook");


        Arm = hardwareMap.get(DcMotorEx.class, "Arm");
        ClawServo = hardwareMap.get(Servo.class, "ClawServo");
        ClawRotation = hardwareMap.get(Servo.class, "ClawRotation");
        BeltServo = hardwareMap.get(CRServo.class, "BeltServo");
        Limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //Gyro
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        imu.initialize(parameters);

        //Limelight settings

        //arm positions converted from degree to counts per rotation calculated
        for (int i = 0; i < 6; i++) {
            ArmPositions[i] = (int)Math.round((1527.793876 / 360) * ArmPositions[i]);
        }

        //Variables for drive

        double LeftPower;
        double RightPower;
        double LeftStrafePower;
        double RightStrafePower;

        //Arm Encoder Values
        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(ArmPower);


        //Wait For Game To Start

        waitForStart();

        //Limelight
        Limelight.setPollRateHz(10); // This sets how often we ask Limelight for data (100 times per second)
        Limelight.pipelineSwitch(2);
        Limelight.start();

        //Set Default Motor Positions
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        Winch.setDirection(DcMotor.Direction.REVERSE);
        WinchHook.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm.setDirection(DcMotor.Direction.REVERSE);
        BeltServo.setPower(0);
        ClawServo.setDirection(Servo.Direction.REVERSE);




        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {
            Limelight.start();

            LLResult Result = Limelight.getLatestResult();
            if (Result != null) {
                double[] pythonOutputs = Result.getPythonOutput();
                if (pythonOutputs != null && pythonOutputs.length > 0) {
                    telemetry.addData("Python Output Length:", pythonOutputs.length);
                    telemetry.addData("Raw Angle from Python Output:", pythonOutputs[0]);
                    Angle = pythonOutputs[0]/150;
                    telemetry.addData("Claw Rotation:", Angle);
                } else {
                    Angle = 16; // Or some other default to indicate no target detected
                    telemetry.addData("Error:", "No angle data received");
                }
                telemetry.update();

            }
            //Joy Stick Values for Driving
            telemetry.addData("Arm Encoder Value", Winch.getCurrentPosition());
            telemetry.update();
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;

            //Field Centric Controls
            if (this.gamepad1.right_bumper) {
                LeftPower = Range.clip(drive + turn, -1.0, 1.0) * .75;
                RightPower = Range.clip(drive - turn, -1.0, 1.0) * .75;
                LeftStrafePower = strafe * .75;
                RightStrafePower = strafe * .75;


                // Send calculated power to wheels
                FrontLeft.setPower(LeftPower);
                FrontRight.setPower(RightPower);
                BackLeft.setPower(LeftPower);
                BackRight.setPower(RightPower);

                if (Math.abs(strafe) > 0.1) {
                    FrontLeft.setPower(LeftStrafePower);
                    FrontRight.setPower(-RightStrafePower);
                    BackLeft.setPower(-LeftStrafePower);
                    BackRight.setPower(RightStrafePower);
                }
            } else {
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

            //Arm Encoder Reset
            if (this.gamepad2.a) {
                runtime.reset();
                Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Arm.setDirection(DcMotor.Direction.REVERSE);
                while (runtime.seconds() < 3) {

                }
                Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setTargetPosition(0);
                Arm.setPower(ArmPower);

            }

            //Gyroscope

            if (this.gamepad1.right_trigger == 1) {
                imu.resetYaw();
                telemetry.addData("Gyroscope", "Reset");
                telemetry.update();

            }

            //Toggle Button for Claw

            if (this.gamepad1.left_trigger > .5) {

                ClawServo.setPosition(.69);


            }
            if (this.gamepad1.left_bumper) {
                ClawServo.setPosition(0);
            }
            if (this.gamepad1.dpad_right) {
                ClawRotation.setPosition(Angle);
            }
            if (this.gamepad1.dpad_up) {
                ClawRotation.setPosition(0);
            }
            if (this.gamepad1.dpad_left) {
                ClawRotation.setPosition(.22);
            }



            //Control for Belt Servo
            if (this.gamepad2.left_trigger == 1) {
                BeltServo.setPower(BeltPower);
            } else if (this.gamepad2.left_bumper) {
                BeltServo.setPower(-BeltPower);
            } else {
                BeltServo.setPower(0);
            }

            //Winch

            if (this.gamepad2.right_trigger == 1) {
                Winch.setDirection(DcMotor.Direction.FORWARD);
                Winch.setPower(WinchPower);
            } else if (this.gamepad2.right_bumper) {
                Winch.setDirection(DcMotor.Direction.REVERSE);
                Winch.setPower(WinchPower);
            } else {
                Winch.setPower(0);
            }

            //Hook Winch

            if (this.gamepad2.dpad_down) {
                WinchHook.setDirection(DcMotor.Direction.FORWARD);
                WinchHook.setPower(WinchPower);
            } else if (this.gamepad2.dpad_up) {
                WinchHook.setDirection(DcMotor.Direction.REVERSE);
                WinchHook.setPower(WinchPower);
            } else {
                WinchHook.setPower(0);
            }

            //Arm

            if (this.gamepad1.y) {
                Arm.setPower(.25);
                Arm.setTargetPosition(ArmPositions[0]);
            } else if (this.gamepad1.b) {
                Arm.setPower(.25);
                Arm.setTargetPosition(ArmPositions[1]);
            } else if (this.gamepad1.a) {
                Arm.setPower(.25);
                Arm.setTargetPosition(ArmPositions[2]);
            } else if (this.gamepad1.x) {
                Arm.setPower(.85);
                Arm.setTargetPosition(ArmPositions[3]);
            }
            else if (this.gamepad1.dpad_down) {
                Arm.setPower(.25);
                Arm.setTargetPosition(ArmPositions[4]);
            }
            else if (this.gamepad2.y) {
                Arm.setPower(.25);
                Arm.setTargetPosition(ArmPositions[5]);
            }
        }
    }
}


