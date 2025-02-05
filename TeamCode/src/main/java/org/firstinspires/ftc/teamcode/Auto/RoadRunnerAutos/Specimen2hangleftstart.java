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
@Autonomous(name = "Specimen2hangleftstart", group = "Autonomous")
public class Specimen2hangleftstart extends LinearOpMode {
    Servo claw2;

    //should be tuned idk figure out what everything is im too lazy rn same rules apply as the bucket auto code -_- jack

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, -66, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        claw2 = hardwareMap.get(Servo.class, "ClawServo");

        TrajectoryActionBuilder Forward = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0,-54.25));
        TrajectoryActionBuilder wait = drive.actionBuilder(initialPose)
                .waitSeconds(1);
        TrajectoryActionBuilder MovetoGrab = drive.actionBuilder(new Pose2d(0,-57, Math.toRadians(90)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(0,-58))
                .strafeTo(new Vector2d(24,-58))
                .turnTo(180);
        TrajectoryActionBuilder Backup =  drive.actionBuilder(new Pose2d(28, -58, Math.toRadians(270)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(24, -58));
        TrajectoryActionBuilder Grab = drive.actionBuilder(new Pose2d(28,-56, Math.toRadians(270)))
                .waitSeconds(5)
                .strafeTo(new Vector2d(24, -61));
        TrajectoryActionBuilder Backup2 =  drive.actionBuilder(new Pose2d(28, -57.5, Math.toRadians(270)))
                .waitSeconds(2)
                .strafeTo(new Vector2d(24, -57));
        TrajectoryActionBuilder secondhang = drive.actionBuilder(new Pose2d(28,-57, Math.toRadians(90)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(24,-58))
                .strafeTo(new Vector2d(-12,-58))
                .waitSeconds(1);
        TrajectoryActionBuilder secondhangforward = drive.actionBuilder(new Pose2d(-10,-58, Math.toRadians(90)))
                .strafeTo(new Vector2d(-12,-54.25))
                .waitSeconds(1);
        TrajectoryActionBuilder backup3 = drive.actionBuilder(new Pose2d(-10, -56, Math.toRadians(90)))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-12, -59));
        TrajectoryActionBuilder Park = drive.actionBuilder(new Pose2d(-10,-59, Math.toRadians(90)))
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
