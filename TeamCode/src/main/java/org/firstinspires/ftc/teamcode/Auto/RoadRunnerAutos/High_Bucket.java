package org.firstinspires.ftc.teamcode.Auto.RoadRunnerAutos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.RoadRunnerAutos.RoadRunnerActions.Claw;
import org.firstinspires.ftc.teamcode.Auto.RoadRunnerAutos.RoadRunnerActions.Arm;
import org.firstinspires.ftc.teamcode.Auto.RoadRunnerAutos.RoadRunnerActions.platform_Climber;


@Config
@Autonomous(name = "High_Bucket", group = "Autonomous")
public class High_Bucket extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-7.5, -66, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        platform_Climber platform = new platform_Climber(hardwareMap);


        TrajectoryActionBuilder Forward = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-7.5,-55));
        TrajectoryActionBuilder Wait = drive.actionBuilder(initialPose)
                .waitSeconds(1);
        TrajectoryActionBuilder Backupafterfirsthang = drive.actionBuilder(new Pose2d(-7.5, -53, Math.toRadians(90)))
                .strafeTo(new Vector2d(-7.5, -60))
                .strafeTo(new Vector2d(-34.5, -60));
        TrajectoryActionBuilder forwardfirstgrab = drive.actionBuilder(new Pose2d(-34.5, -52, Math.toRadians(90)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-34.5, -49.5));
        TrajectoryActionBuilder Wait1 = drive.actionBuilder(new Pose2d(-34.5, -49.5, Math.toRadians(90)))
                .waitSeconds(1);




        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        claw.Close(),
                        Wait.build(),
                        arm.HookPos(),
                        Forward.build(),
                        arm.Hook(),
                        Backupafterfirsthang.build(),
                        arm.Pickup(),
                        forwardfirstgrab.build(),
                        claw.Close(),
                        Wait1.build(),
                        platform.Climbhigh()
                )
        );

    }
}