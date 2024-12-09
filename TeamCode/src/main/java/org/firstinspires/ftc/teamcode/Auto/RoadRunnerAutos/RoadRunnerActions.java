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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "RoadRunnerActions")
public class RoadRunnerActions extends LinearOpMode {
    public static class platform_Climber {
        CRServo Platform;
        DcMotor Climber;
        public platform_Climber(HardwareMap HardwareMap) {
            Platform = HardwareMap.get(CRServo.class, "BeltServo");
            Climber = HardwareMap.get(DcMotor.class, "Winch");
            Climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Climber.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public class ClimberStartPos implements Action {
            boolean PosReached = true;
            public boolean run(@NonNull TelemetryPacket packet) {
                Climber.setPower(1);
                if (Climber.getCurrentPosition() > 5540) {
                    Climber.setPower(0);
                    PosReached = false;
                }
                return PosReached;
            }
        }
        public Action Climbhigh(){return new platform_Climber.ClimberStartPos();}
        public class Platformpowered implements Action {
            boolean powered = true;
            public boolean run(@NonNull TelemetryPacket packet) {
                Platform.setPower(1);
                powered = false;
                return powered;
            }
        }
        public Action Platformpowered() {
            return new platform_Climber.Platformpowered();
        }
        public class Platformunpowered implements Action {
            boolean powered = true;
            public boolean run(@NonNull TelemetryPacket packet) {
                Platform.setPower(0);
                powered = false;
                return powered;
            }
        }
        public Action Platformunpowered() {
            return new platform_Climber.Platformpowered();
        }
    }
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
            claw = HardwareMap.get(Servo.class, "ClawServo");
            claw.setDirection(Servo.Direction.REVERSE);

        }
        public class  start implements Action  {
            boolean grabbed = true;
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setTargetPosition(50);
                if (arm.getCurrentPosition() > 45) {
                    grabbed = false;
                }
                return grabbed;
            }
        }
        public Action start(){return new Arm.start();}
        public class Pickup implements Action {
            boolean PosReached = true;
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setPower(.25);
                arm.setTargetPosition(15);
                if (arm.getCurrentPosition() < 40) {
                    PosReached = false;
                }
                return PosReached;
            }
        }
        public Action Pickup() {
            return new Arm.Pickup();
        }

        public class B implements Action {
            boolean PosReached = true;
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setPower(.25);
                arm.setTargetPosition(130);
                if (arm.getCurrentPosition() > 120) {
                    PosReached = false;
                }
                return PosReached;
            }
        }
        public Action B(){return new Arm.B();}

        public class Pickupbucket implements Action {
            boolean PosReached = true;
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setPower(.25);
                arm.setTargetPosition(10);
                if (arm.getCurrentPosition() < 25) {
                    PosReached = false;
                }
                return PosReached;
            }
        }
        public Action Pickupbucket() {
            return new Arm.Pickupbucket();
        }

        public class MovPickup implements Action {
            boolean PosReached = true;

            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setPower(.25);
                arm.setTargetPosition(35);
                if (arm.getCurrentPosition() < 90) {
                    PosReached = false;
                }
                return PosReached;
            }
        }


        public class Bucket implements Action {
            boolean PosReached = true;
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setPower(.25);
                arm.setTargetPosition(467);
                if (arm.getCurrentPosition() >457) {
                    PosReached = false;
                }
                return PosReached;
            }
        }
        public Action Bucket() {
            return new Arm.Bucket();
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
                        claw.setPosition(0);
                        if (claw.getPosition() < .2) {
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
        private Servo Rotation;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "ClawServo");
            claw.setDirection(Servo.Direction.REVERSE);
            Rotation = hardwareMap.get(Servo.class, "ClawRotation");
        }
        public class Rotate implements Action {
            boolean rotated = true;
            public boolean run(@NonNull TelemetryPacket packet) {
                Rotation.setPosition(.2);
                rotated = false;
                return rotated;
            }
        }
        public class Rotate0 implements Action {
            boolean rotated = true;
            public boolean run(@NonNull TelemetryPacket packet) {
                Rotation.setPosition(1);
                rotated = false;
                return rotated;
            }
        }

        public Action Rotate() {
            return new Claw.Rotate();
        }
        public Action Rotate0() {
            return new Claw.Rotate0();
        }

        public class Open implements Action {
            boolean Open = false;

            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0);
                if (claw.getPosition() < .1) {
                    Open = false;
                }
                return Open;
            }
        }

        public class Close implements Action {
            boolean Close = true;
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(.72);
                if (claw.getPosition() > .1) {
                    Close = false;
                }
                return Close;
            }
        }

        public Action Open() {return new Claw.Open();}

        public Action Close() {
            return new Claw.Close();
        }
    }
    @Override
    public void runOpMode() {
    }
}

