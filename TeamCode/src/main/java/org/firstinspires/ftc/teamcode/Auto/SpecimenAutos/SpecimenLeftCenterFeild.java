package org.firstinspires.ftc.teamcode.Auto.SpecimenAutos;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.Auto.Config.AutonomousCommands;




//Change to File Name
@Autonomous(name = "SpecimenLeftCenterFeild")
//Change to File Name
public class SpecimenLeftCenterFeild extends LinearOpMode {
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
            AutonomousCommands.MoveArm(1,.1,.25,Arm);
            AutonomousCommands.Claw(.5,ClawServo);
            AutonomousCommands.Forward(1.34, .4, DriveMotors);
            AutonomousCommands.Stop(DriveMotors);
            AutonomousCommands.MoveArm(2,.34,.75,Arm);
            AutonomousCommands.Claw(1,ClawServo);
            AutonomousCommands.Backward(.5, .4, DriveMotors);
            AutonomousCommands.Strafe(2.35, .35, DriveMotors);
            AutonomousCommands.Stop(DriveMotors);
            AutonomousCommands.MoveArm(0,5,.2,Arm);
            AutonomousCommands.Forward(.5, .4, DriveMotors);
            AutonomousCommands.Stop(DriveMotors);
            AutonomousCommands.Claw(.5,ClawServo);
            AutonomousCommands.MoveArm(2,5,.2,Arm);
            AutonomousCommands.Turn(1.4, .3, DriveMotors);
            AutonomousCommands.Forward(.8, .4, DriveMotors);
            AutonomousCommands.Stop(DriveMotors);
            AutonomousCommands.Claw(1,ClawServo);
            AutonomousCommands.Backward(.5, .4, DriveMotors);
            AutonomousCommands.Stop(DriveMotors);
            AutonomousCommands.Wait(.2);
            AutonomousCommands.Forward(.3, .1, DriveMotors);
            AutonomousCommands.Stop(DriveMotors);
            AutonomousCommands.Claw(.5,ClawServo);
            AutonomousCommands.MoveArm(1,.4,.25,Arm);
            AutonomousCommands.Backward(.6, .1, DriveMotors);
            AutonomousCommands.Turn(1.4, -.3, DriveMotors);
            AutonomousCommands.Strafe(2.35, -.35, DriveMotors);
            AutonomousCommands.Forward(.65, .4,DriveMotors);
            AutonomousCommands.Stop(DriveMotors);
            AutonomousCommands.MoveArm(2,.4,.75,Arm);
            AutonomousCommands.Claw(2,ClawServo);
            AutonomousCommands.Backward(1.25, .1, DriveMotors);
            AutonomousCommands.Strafe(2.35, -.35, DriveMotors);
            AutonomousCommands.Stop(DriveMotors);
            break;
        }
    }
}
