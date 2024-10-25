package org.firstinspires.ftc.teamcode.Auto.RoadRunnerAutos;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "RoadRunnerTest", group = "Autonomous")
public class RoadRunnerTest extends LinearOpMode {
    public class Arm {
        DcMotor arm;
        Servo claw;

        public Arm(HardwareMap HardwareMap) {
            arm = HardwareMap.get(DcMotor.class, "Arm");
            arm.setDirection(DcMotor.Direction.REVERSE);
            arm.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(.5);
            claw = HardwareMap.get(Servo.class, "ArmServo");
        }

        public class Pickup implements Action {
            boolean PosReached = false;

            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setPower(.5);
                arm.setTargetPosition(0);
                if (arm.getCurrentPosition() == arm.getTargetPosition()) {
                    PosReached = true;
                }
                return PosReached;
            }
        }

        public Action Pickup() {
            return new Pickup();
        }

        public class Hook implements Action {
            boolean Hooked = false;

            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setPower(.5);
                arm.setTargetPosition(90);
                if (arm.getCurrentPosition() == arm.getTargetPosition()) {
                    arm.setPower(.85);
                    arm.setTargetPosition(50);
                }
                if (arm.getCurrentPosition() < 65) {
                    claw.setPosition(1);
                    Hooked = true;
                }
                return Hooked;
            }
        }
        public Action Hook() {
            return new Hook();
        }
        public class HookPos implements Action {
            boolean posreached = false;
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setTargetPosition(90);
                if (arm.getCurrentPosition() == 90) {
                    posreached = true;
                }
                return posreached;
            }
        }
        public Action HookPos() {
            return new HookPos();
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "ArmServo");
        }

        public class Open implements Action {
            boolean Open = false;

            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1);
                if (claw.getPosition() == 1) {
                    Open = true;
                }
                return Open;
            }
        }

        public class Close implements Action {
            boolean Closed = false;

            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1);
                if (claw.getPosition() == 1) {
                    Closed = true;
                }
                return Closed;
            }
        }

        public Action Open() {

            return new Open();
        }

        public Action Close() {

            return new Close();
        }
    }
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, -66, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        int VisionOutputPosition = 1;

        TrajectoryActionBuilder Forward = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0,-56));
        TrajectoryActionBuilder MovetoGrab = Forward.fresh()
                .strafeTo(new Vector2d(0,-62))
                .turnTo(180)
                .strafeTo(new Vector2d(48,-62))
                .waitSeconds(5)
                .strafeTo(new Vector2d(48, -66));
        TrajectoryActionBuilder secondhang = MovetoGrab.fresh()
                .turnTo(0)
                .strafeTo(new Vector2d(0,-56));
        TrajectoryActionBuilder Park = secondhang.fresh()
                .strafeTo(new Vector2d(65,-60));





        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        arm.HookPos(),
                        claw.Open(),
                        claw.Close(),
                        arm.Hook()
                )
        );

    }
}
