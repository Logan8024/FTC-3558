package org.firstinspires.ftc.teamcode.Teleop.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class JoyStick {
    public static void Command(boolean button, boolean goal, Runnable action) {
        if (button == goal) {
            action.run();
        }
    }
    public static void Command(double button, boolean goal, Runnable action) {
        if (button > .5 == goal) {
            action.run();
        }
    }
    public static class JoyStick1 {
        public static Gamepad Joystick1 = hardwareMap.get(Gamepad.class, "gamepad1");

        public static int ID() {
            return Joystick1.getGamepadId();
        }
        public static class LeftStick {
            public static double x = Joystick1.left_stick_x;
            public static double y = Joystick1.left_stick_y;
        }
        public static class RightStick {
            public static double x = Joystick1.right_stick_x;
            public static double y =  Joystick1.right_stick_y;
        }
        public static double LeftTrigger = Joystick1.left_trigger;
        public static double RightTrigger = Joystick1.right_trigger;


        public static boolean RightBumper = Joystick1.right_bumper;
        public static boolean LeftBumper = Joystick1.left_bumper;

        public static boolean DpadUp = Joystick1.dpad_up;
        public static  boolean DpadDown = Joystick1.dpad_down;
        public static boolean DpadLeft = Joystick1.dpad_left;
        public static boolean DpadRight = Joystick1.dpad_right;

        public static boolean A = Joystick1.a;
        public static boolean B = Joystick1.b;
        public static boolean X = Joystick1.x;
        public static boolean Y = Joystick1.y;
    }
    public static class JoyStick2 {
        static Gamepad Joystick2 = new Gamepad();
        public static int ID() {
            return Joystick2.getGamepadId();
        }
        public static class LeftStick {
            public static double x = Joystick2.left_stick_x;
            public static double y = Joystick2.left_stick_y;
        }
        public static class RightStick {
            public static double x = Joystick2.right_stick_x;
            public static double y =  Joystick2.right_stick_y;
        }
        public static double LeftTrigger = Joystick2.left_trigger;
        public static double RightTrigger = Joystick2.right_trigger;


        public static boolean RightBumper = Joystick2.right_bumper;
        public static boolean LeftBumper = Joystick2.left_bumper;

        public static boolean DpadUp = Joystick2.dpad_up;
        public static  boolean DpadDown = Joystick2.dpad_down;
        public static boolean DpadLeft = Joystick2.dpad_left;
        public static boolean DpadRight = Joystick2.dpad_right;

        public static boolean A = Joystick2.a;
        public static boolean B = Joystick2.b;
        public static boolean X = Joystick2.x;
        public static boolean Y = Joystick2.y;
    }
}
