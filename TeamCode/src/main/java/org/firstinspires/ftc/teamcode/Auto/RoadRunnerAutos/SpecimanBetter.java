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
@Autonomous(name = "Specimenbetter", group = "Autonomous")
public class SpecimanBetter extends LinearOpMode {
    Servo claw2;

    //should be tuned idk figure out what everything is im too lazy rn same rules apply as the bucket auto code -_- jack

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(7, -65, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        claw2 = hardwareMap.get(Servo.class, "ClawServo");
        platform_Climber platform = new RoadRunnerActions.platform_Climber(hardwareMap);
        TrajectoryActionBuilder wait = drive.actionBuilder(initialPose)
                .waitSeconds(3);
        TrajectoryActionBuilder wait2 = drive.actionBuilder(initialPose)
                .waitSeconds(1);
        TrajectoryActionBuilder catinthehat = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(7, -64.75));
        TrajectoryActionBuilder Forwardtohang = drive.actionBuilder(new Pose2d(7,-64.5, Math.toRadians(90)))
                .strafeTo(new Vector2d(7,-55.7));
        TrajectoryActionBuilder backupafterhang = drive.actionBuilder(new Pose2d(7,-54.25, Math.toRadians(90)))
                .strafeTo(new Vector2d(7,-60))
                .strafeTo(new Vector2d(21,-60))
                .strafeTo(new Vector2d(21,-45))
                .strafeTo(new Vector2d(24,-45))
                .strafeTo(new Vector2d(24,-67))
                .strafeTo(new Vector2d(28,-60))
                .turnTo(Math.toRadians(270))
                .strafeTo(new Vector2d(32, -65));
        TrajectoryActionBuilder Wait5 = drive.actionBuilder(new Pose2d(32,-66, Math.toRadians(270)))
                        .waitSeconds(3);
        TrajectoryActionBuilder MovetoGrab = drive.actionBuilder(new Pose2d(32,-66, Math.toRadians(270)))
                .strafeTo(new Vector2d(32, -68));
        TrajectoryActionBuilder Wait1 = drive.actionBuilder(new Pose2d(32,-68, Math.toRadians(270)))
                .waitSeconds(1);
        TrajectoryActionBuilder backupaftergrab = drive.actionBuilder(new Pose2d(32,-68, Math.toRadians(270)))
                .strafeTo(new Vector2d(32,-64.5) );
        TrajectoryActionBuilder MovetoHang = drive.actionBuilder(new Pose2d(32,-64.5, Math.toRadians(270)))
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(-2, -52.5));
        TrajectoryActionBuilder backup = drive.actionBuilder(new Pose2d(-2,-54.5, Math.toRadians(90)))
                .strafeTo(new Vector2d(-2, -62));
        TrajectoryActionBuilder wait4 = drive.actionBuilder(new Pose2d(-2,-62, Math.toRadians(90)))
                        .waitSeconds(1);
        







        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        arm.Pickup(),
                        wait2.build(),
                        claw.Close(),
                        wait.build(),
                        arm.HookPos(),
                        Forwardtohang.build(),
                        arm.Hook(),
                        backupafterhang.build(),
                        arm.Pickup(),
                        Wait5.build(),
                        MovetoGrab.build(),
                        claw.Close(),
                        Wait1.build(),
                        backupaftergrab.build(),
                        arm.HookPos(),
                        MovetoHang.build(),
                        arm.Hook(),
                        backup.build(),
                        claw.Close(),
                        wait4.build()



                )
        );

    }
}