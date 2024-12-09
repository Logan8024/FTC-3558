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
                .strafeTo(new Vector2d(-7.5,-54.75));
        TrajectoryActionBuilder Wait = drive.actionBuilder(initialPose)
                .waitSeconds(1);
        TrajectoryActionBuilder Backupafterfirsthang = drive.actionBuilder(new Pose2d(-7.5, -54.75, Math.toRadians(90)))
                .strafeTo(new Vector2d(-7.5, -60))
                .strafeTo(new Vector2d(-34.5, -60));
        TrajectoryActionBuilder forwardfirstgrab = drive.actionBuilder(new Pose2d(-34.5, -60, Math.toRadians(90)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-34.5, -51));
        TrajectoryActionBuilder Wait1 = drive.actionBuilder(new Pose2d(-34.5, -51, Math.toRadians(90)))
                .waitSeconds(1);
        TrajectoryActionBuilder Wait3 = drive.actionBuilder(new Pose2d(-34.5, -51, Math.toRadians(90)))
                .waitSeconds(3.25);
        TrajectoryActionBuilder BackupafterFirstgrab = drive.actionBuilder(new Pose2d(-34.5, -51, Math.toRadians(90)))
                .strafeTo(new Vector2d(-34.5, -60));
        TrajectoryActionBuilder TurnAfterFirstGrab = drive.actionBuilder(new Pose2d(-34.5, -60, Math.toRadians(90)))
                        .turnTo(Math.toRadians(210));
        TrajectoryActionBuilder forwardtobucket = drive.actionBuilder(new Pose2d(-34.5, -60, Math.toRadians(210)))
                .strafeTo(new Vector2d(-38, -66));
        TrajectoryActionBuilder wait4 = drive.actionBuilder(new Pose2d(-38, -66, Math.toRadians(210)))
                .waitSeconds(1);
        TrajectoryActionBuilder Backupafterbucket = drive.actionBuilder(new Pose2d(-38, -66, Math.toRadians(210)))
                        .strafeTo(new Vector2d(-34.5, -60))
                        .turnTo(Math.toRadians(90));




        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        claw.Close(),
                        Wait.build(),
                        arm.HookPos(),
                        Forward.build(),
                        arm.Hook(),
                        Backupafterfirsthang.build(),
                        arm.MovPickup(),
                        forwardfirstgrab.build(),
                        arm.Pickupbucket(),
                        claw.Close(),
                        Wait1.build(),
                        arm.Bucket(),
                        platform.Climbhigh(),
                        platform.Platformpowered(),
                        claw.Rotate(),
                        Wait3.build(),
                        platform.Platformunpowered(),
                        BackupafterFirstgrab.build(),
                        TurnAfterFirstGrab.build(),
                        forwardtobucket.build(),
                        claw.Rotate0(),
                        wait4.build(),
                        claw.Open(),
                        wait4.build(),
                        claw.Rotate(),
                        wait4.build(),
                        arm.B(),
                        Backupafterbucket.build()

                )
        );

    }
}