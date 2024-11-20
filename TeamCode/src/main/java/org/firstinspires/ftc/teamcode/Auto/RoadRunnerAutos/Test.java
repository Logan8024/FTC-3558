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


@Config
@Autonomous(name = "TestAuto", group = "Autonomous")
public class Test extends LinearOpMode {
    Servo claw2;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, -66, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        claw2 = hardwareMap.get(Servo.class, "ClawServo");

        TrajectoryActionBuilder Forward = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0,-57));

        TrajectoryActionBuilder wait = drive.actionBuilder(initialPose)
                .waitSeconds(1);

        TrajectoryActionBuilder MovetoGrab = Forward.endTrajectory()
                .waitSeconds(1)
                .strafeTo(new Vector2d(0,-58))
                .strafeTo(new Vector2d(28,-58))
                .turnTo(180);

        TrajectoryActionBuilder Backup =  MovetoGrab.endTrajectory()
                .waitSeconds(1)
                .strafeTo(new Vector2d(28, -56));

        TrajectoryActionBuilder Grab = Backup.endTrajectory()
                .waitSeconds(5)
                .strafeTo(new Vector2d(28, -57.5));

        TrajectoryActionBuilder Backup2 =  Grab.endTrajectory()
                .waitSeconds(2)
                .strafeTo(new Vector2d(28, -57));

        TrajectoryActionBuilder secondhang = Backup2.endTrajectory()
                .waitSeconds(1)
                .strafeTo(new Vector2d(28,-58))
                .strafeTo(new Vector2d(-12,-58))
                .waitSeconds(1);

        TrajectoryActionBuilder secondhangforward = secondhang.endTrajectory()
                .strafeTo(new Vector2d(-12,-56.75))
                .waitSeconds(1);

        TrajectoryActionBuilder backup3 = secondhangforward.endTrajectory()
                .waitSeconds(1)
                .strafeTo(new Vector2d(-12, -59));
        
        TrajectoryActionBuilder Park = backup3.endTrajectory()
                .strafeTo(new Vector2d(28, -63));




        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        claw.Close(),
                        wait.build(),
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
