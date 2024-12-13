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

    //Climber and Platform Autonomous Commands
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

        //Move Climber to Highest Position
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

        //Move Climber to Lowest Position
        public class ClimberEndPos implements Action {
            boolean PosReached = true;
            public boolean run(@NonNull TelemetryPacket packet) {
                Climber.setPower(-1);
                if (Climber.getCurrentPosition() < 80) {
                    Climber.setPower(0);
                    PosReached = false;
                }
                return PosReached;
            }
        }
        public Action ClimberEndPos(){return new platform_Climber.ClimberEndPos();}

        //Power the Platform
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

        //Unpower the Platform
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

    //Arm Autonomous Actions
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

        //Position for the Start of the Autonmous moves Arm to position above the specimen hanging bar
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

        //Position for picking up a specimen
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

        //Highest Arm Positon Used for after Scoring in the bucket to put arm all the way back
        public class B implements Action {
            boolean PosReached = true;
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setPower(.25);
                arm.setTargetPosition(733);
                if (arm.getCurrentPosition() > 720) {
                    PosReached = false;
                }
                return PosReached;
            }
        }
        public Action B(){return new Arm.B();}

        //Picking up samples
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

        //Slightly higher arm position for moving around before picking up a game piece
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

        //Arm position for scoring in the bucket
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

        //Arm Function that hooks the specimen only call when robot is in correct position this function includes claw actions
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

        //position of arm for hooking? or maybe when driving up to it honestly i cant remember
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

    //Claw actions, DUH
    public static class Claw {
        private Servo claw;
        private Servo Rotation;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "ClawServo");
            claw.setDirection(Servo.Direction.REVERSE);
            Rotation = hardwareMap.get(Servo.class, "ClawRotation");
        }

        //Rotate to 90 degrees
        public class Rotate implements Action {
            boolean rotated = true;
            public boolean run(@NonNull TelemetryPacket packet) {
                Rotation.setPosition(.3);
                rotated = false;
                return rotated;
            }
        }

        //Rotate to 0 degrees
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

        //self explanatory
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

        //self explanatory
        public class Close implements Action {
            boolean Close = true;
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(.8);
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

