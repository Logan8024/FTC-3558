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

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "RoadRunnerActions")
public class RoadRunnerActions extends LinearOpMode {
    public static class Arm {
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
                arm.setPower(.25);
                arm.setTargetPosition(0);
                if (arm.getCurrentPosition() < 40) {
                    PosReached = false;
                }
                return PosReached;
            }
        }
        public Action Pickup() {
            return new Arm.Pickup();
        }
        public class MovPickup implements Action {
            boolean PosReached = true;

            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setPower(.25);
                arm.setTargetPosition(60);
                if (arm.getCurrentPosition() < 90) {
                    PosReached = false;
                }
                return PosReached;
            }
        }

        public Action MovPickup() {
            return new Arm.MovPickup();
        }

        public class Hook implements Action {
            boolean Hooked = true;
            boolean FirstPosReached = false;
            boolean FailSafe = false;
            ElapsedTime runtime = new ElapsedTime();

            public boolean run(@NonNull TelemetryPacket packet) {
                runtime.reset();
                if (!FailSafe) {
                    if (arm.getCurrentPosition() == 0) {
                        arm.setPower(.25);
                        arm.setTargetPosition(382);
                    }
                    if (arm.getCurrentPosition() > 372 && arm.getCurrentPosition() < 402) {
                        arm.setPower(.75);
                        arm.setTargetPosition(212);
                        FirstPosReached = true;
                    }
                    if (arm.getCurrentPosition() < 265 && FirstPosReached && Hooked) {
                        claw.setPosition(1);
                        if (claw.getPosition() > .8) {
                            Hooked = false;
                        }
                    }
                }
                if (runtime.seconds() > 10 && FirstPosReached) {
                    FailSafe = true;
                }
                if (FailSafe) {
                    arm.setTargetPosition(382);
                    if (arm.getCurrentPosition() > 320) {
                        Hooked = false;
                    }
                }
                return Hooked;
            }
        }
        public Action Hook() {
            return new Arm.Hook();
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
            return new Arm.HookPos();
        }
    }

    public static class Claw {
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
                if (claw.getPosition() < .8) {
                    Close = false;
                }
                return Close;
            }
        }

        public Action Open() {

            return new Claw.Open();
        }

        public Action Close() {
            return new Claw.Close();
        }
    }
    @Override
    public void runOpMode() {
    }
}

