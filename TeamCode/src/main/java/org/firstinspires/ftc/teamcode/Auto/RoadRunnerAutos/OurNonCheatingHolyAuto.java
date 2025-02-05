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

import org.firstinspires.ftc.teamcode.RoadRunnerConfig.MecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.RoadRunnerAutos.RoadRunnerActions.Claw;
import org.firstinspires.ftc.teamcode.Auto.RoadRunnerAutos.RoadRunnerActions.Arm;


@Config
@Autonomous(name = "CatInTheHat", group = "Autonomous")
public class OurNonCheatingHolyAuto extends LinearOpMode {
    Servo claw2;

    //Auto for cat in the hat is not functional
    //due to climber update but the start code is useful in the fact that you can copy it if needed
    // and tune it for either bucket or specimen

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, -66, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        claw2 = hardwareMap.get(Servo.class, "ClawServo");

        TrajectoryActionBuilder oneforward = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0,-65.75));

        //Dont copy past this line
        TrajectoryActionBuilder Forward = drive.actionBuilder(new Pose2d(0,-65.75, Math.toRadians(90)))
                .strafeTo(new Vector2d(0,-57));
        TrajectoryActionBuilder wait = drive.actionBuilder(new Pose2d(0,-65.75, Math.toRadians(90)))
                .waitSeconds(1);
        TrajectoryActionBuilder MovetoGrab = drive.actionBuilder(new Pose2d(0,-57, Math.toRadians(90)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(0,-58))
                .strafeTo(new Vector2d(28,-58))
                .turnTo(180);
        TrajectoryActionBuilder Backup =  drive.actionBuilder(new Pose2d(28, -58, Math.toRadians(270)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(28, -56));
        TrajectoryActionBuilder Grab = drive.actionBuilder(new Pose2d(28,-56, Math.toRadians(270)))
                .waitSeconds(5)
                .strafeTo(new Vector2d(28, -57.5));
        TrajectoryActionBuilder Backup2 =  drive.actionBuilder(new Pose2d(28, -57.5, Math.toRadians(270)))
                .waitSeconds(2)
                .strafeTo(new Vector2d(28, -57));
        TrajectoryActionBuilder secondhang = drive.actionBuilder(new Pose2d(28,-57, Math.toRadians(90)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(28,-58))
                .strafeTo(new Vector2d(-10,-58))
                .waitSeconds(1);
        TrajectoryActionBuilder secondhangforward = drive.actionBuilder(new Pose2d(-10,-58, Math.toRadians(90)))
                .strafeTo(new Vector2d(-10,-56.5))
                .waitSeconds(1);
        TrajectoryActionBuilder backup3 = drive.actionBuilder(new Pose2d(-10, -56, Math.toRadians(90)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-10, -59));
        TrajectoryActionBuilder Park = drive.actionBuilder(new Pose2d(-10,-59, Math.toRadians(90)))
                .strafeTo(new Vector2d(28, -63));




        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        arm.start(),
                        oneforward.build(),
                        claw.Close(),
                        wait.build(), // dont copy past this line if need be because then it gets to actual code
                        arm.HookPos(),
                        Forward.build(),
                        arm.Hook(),
                        MovetoGrab.build(),
                        Backup.build(),
                        arm.MovPickup(),
                        Grab.build(),
                        arm.Pickup(),
                        claw.Close(),
                        Backup2.build(),
                        arm.HookPos(),
                        secondhang.build(),
                        secondhangforward.build(),
                        arm.Hook(),
                        backup3.build(),
                        Park.build()

                )
        );

    }
}
