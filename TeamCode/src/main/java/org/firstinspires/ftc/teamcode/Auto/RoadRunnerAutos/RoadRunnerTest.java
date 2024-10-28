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
            boolean PosReached = true;

            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setPower(.5);
                arm.setTargetPosition(0);
                if (arm.getCurrentPosition() == arm.getTargetPosition()) {
                    PosReached = false;
                }
                return PosReached;
            }
        }

        public Action Pickup() {
            return new Pickup();
        }

        public class Hook implements Action {
            boolean Hooked = true;
            boolean PosReached = false;

            public boolean run(@NonNull TelemetryPacket packet) {
                if (arm.getCurrentPosition() == 0){
                    arm.setPower(.25);
                    arm.setTargetPosition(382);
                }
                if (arm.getCurrentPosition() > 372 && arm.getCurrentPosition() < 402) {
                    arm.setPower(.75);
                    arm.setTargetPosition(212);
                    PosReached = true;
                }
                if (arm.getCurrentPosition() < 265 && PosReached) {
                    claw.setPosition(1);
                    if (claw.getPosition() < .8) {
                        Hooked = false;
                    }
                }
                return Hooked;
            }
        }
        public Action Hook() {
            return new Hook();
        }
        public class HookPos implements Action {
            boolean posreached = true;
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setTargetPosition(382);
                if (arm.getCurrentPosition() > 362 && arm.getCurrentPosition() < 402) {
                    posreached = false;
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
                Open = false;
                return Open;
            }
        }

        public class Close implements Action {
            boolean Close = true;

            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(.62);
                Close = false;
                return Close;
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
                        claw.Close(),
                        arm.Hook()
                )
        );

    }
}

