package org.firstinspires.ftc.teamcode.Auto.RoadRunnerAutos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunnerConfig.MecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.RoadRunnerAutos.RoadRunnerActions.Claw;
import org.firstinspires.ftc.teamcode.Auto.RoadRunnerAutos.RoadRunnerActions.Arm;
import org.firstinspires.ftc.teamcode.Auto.RoadRunnerAutos.RoadRunnerActions.platform_Climber;


@Config
@Autonomous(name = "High_Bucket", group = "Autonomous")
public class High_Bucket extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-8.5, -65, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        platform_Climber platform = new platform_Climber(hardwareMap);

        //1 tick is larger then you believe small adjustments like it not clipping on the hook are usually .25

        //Drive forward to the Bar, Hook action is called after this
        TrajectoryActionBuilder Wait123 = drive.actionBuilder(initialPose)
                .waitSeconds(3);
        TrajectoryActionBuilder Wait124 = drive.actionBuilder(initialPose)
                .waitSeconds(.5);

        TrajectoryActionBuilder Forward = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-8.5,-55.7));

        //Wait for claw to close, Happens at start - Jack don't even think of changing this
        TrajectoryActionBuilder Wait = drive.actionBuilder(initialPose)
                .waitSeconds(1);

        //Back up from specimen and strafe over to where the sample game piece is
        TrajectoryActionBuilder Backupafterfirsthang = drive.actionBuilder(new Pose2d(-7.5, -55.6, Math.toRadians(90)))
                .strafeTo(new Vector2d(-8.5, -58))
                .strafeTo(new Vector2d(-30.25, -58));

        //move forward to the game piece
        TrajectoryActionBuilder forwardfirstgrab = drive.actionBuilder(new Pose2d(-30.25, -60, Math.toRadians(90)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-30.25, -48.5));

        //Wait for closing claw, yes this is necessary, no i cant implement this into the action jack, why, because runtime doesnt work in actions for some reason only god knows why
        TrajectoryActionBuilder Wait1 = drive.actionBuilder(new Pose2d(-33.5, -51, Math.toRadians(90)))
                .waitSeconds(1);

        //wait for platform to move, Same reason as before
        TrajectoryActionBuilder Wait3 = drive.actionBuilder(new Pose2d(-33.5, -51, Math.toRadians(90)))
                .waitSeconds(.75);

        //Back up after game piece is grabbed
        TrajectoryActionBuilder BackupafterFirstgrab = drive.actionBuilder(new Pose2d(-33.5, -51, Math.toRadians(90)))
                .strafeTo(new Vector2d(-30.5, -60));

        //Rotate toward the bucket
        TrajectoryActionBuilder TurnAfterFirstGrab = drive.actionBuilder(new Pose2d(-33.5, -60, Math.toRadians(90)))
                        .turnTo(Math.toRadians(210));

        //Move forward to the bucket
        TrajectoryActionBuilder forwardtobucket = drive.actionBuilder(new Pose2d(-33.5, -60, Math.toRadians(210)))
                .strafeTo(new Vector2d(-30, -64));

        //Wait
        TrajectoryActionBuilder wait4 = drive.actionBuilder(new Pose2d(-30, -64, Math.toRadians(210)))
                .waitSeconds(1);

        //Wait
        TrajectoryActionBuilder wait5 = drive.actionBuilder(new Pose2d(-30, -64, Math.toRadians(210)))
                .waitSeconds(1);

        //Wait
        TrajectoryActionBuilder wait6 = drive.actionBuilder(new Pose2d(-30, -64, Math.toRadians(210)))
                .waitSeconds(1);

        //Backup after the bucket
        TrajectoryActionBuilder Backupafterbucket = drive.actionBuilder(new Pose2d(-38, -66, Math.toRadians(210)))
                        .strafeTo(new Vector2d(-33.5, -56))
                        .turnTo(Math.toRadians(105));

        //Strafe to the side so its aligned with the second piece
        TrajectoryActionBuilder Strafeattheend  = drive.actionBuilder(new Pose2d(-33.5, -60, Math.toRadians(90)))
                .strafeTo(new Vector2d(-38, -56));




        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        claw.Close(),
                        Wait123.build(),
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
                        platform.Platformpowered(),
                        platform.Climbhigh(),
                        claw.Rotate(),
                        Wait3.build(),
                        platform.Platformunpowered(),
                        BackupafterFirstgrab.build(),
                        TurnAfterFirstGrab.build(),
                        forwardtobucket.build(),
                        claw.Rotate0(),
                        wait4.build(),
                        claw.Open(),
                        wait5.build(),
                        claw.Rotate(),
                        claw.Close(),
                        wait6.build(),
                        arm.B(),
                        Backupafterbucket.build(),
                        platform.ClimberEndPos(),
                        Strafeattheend.build(),
                        arm.B()
                )
        );

    }
}