package org.firstinspires.ftc.teamcode.Competition.TeleOp;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public abstract class TeleOpMethods extends TeleOpHardware {

    public void mecanumDrive(){
        final double driveSpeed = 0.66;
        final double fastSpeed = 1.0;
        final double slowSpeed = 0.25;
        double finalSlowmode = 0.0;
        boolean slowMode = true;
        int gyroAdj = 0;

        //Automated time variables
        double current1 = Double.MAX_VALUE;
        double current2 = Double.MAX_VALUE;
        double current3 = Double.MAX_VALUE;
        double current4 = Double.MAX_VALUE;
        double current5 = Double.MAX_VALUE;

        //Lifting and Servo Variables
        int liftPos = 0;
        int liftPosAdj = 0;
        double liftPower = 0.0;
        double clawPos = 0.0;
        double servoRot = 0.0;
        double outtakeRot = -1.0;
        boolean startHanging = false;

        public void getController() {
            //Gamepad joysticks
            g1_leftstick_x = gamepad1.left_stick_x;
            g2_leftstick_x = gamepad2.left_stick_x;

            g1_leftstick_y = gamepad1.left_stick_y;
            g2_leftstick_y = gamepad2.left_stick_y;

            g1_rightstick_x = gamepad1.right_stick_x;
            g2_rightstick_x = gamepad2.right_stick_x;

            g1_rightstick_y = gamepad1.right_stick_y;
            g2_rightstick_y = gamepad2.right_stick_y;


            //Directional pad buttons
            g1_dpad_down = gamepad1.dpad_down;
            g1_dpad_up = gamepad1.dpad_up;
            g1_dpad_right = gamepad1.dpad_right;
            g1_dpad_left = gamepad1.dpad_left;
            g2_dpad_down = gamepad2.dpad_down;
            g2_dpad_up = gamepad2.dpad_up;
            g2_dpad_right = gamepad2.dpad_right;
            g2_dpad_left = gamepad2.dpad_left;

            //Gamepad buttons
            g1_a = gamepad1.a;
            g1_b = gamepad1.b;
            g1_x = gamepad1.x;
            g1_y = gamepad1.y;
            g2_a = gamepad2.a;
            g2_b = gamepad2.b;
            g2_x = gamepad2.x;
            g2_y = gamepad2.y;

            //Gamepad bumpers
            g1_right_bumper = gamepad1.right_bumper;
            g1_left_bumper = gamepad1.left_bumper;
            g2_right_bumper = gamepad2.right_bumper;
            g2_left_bumper = gamepad2.left_bumper;

            //Gamepad triggers
            g1_right_trigger = gamepad1.right_trigger;
            g1_left_trigger = gamepad1.left_trigger;
            g2_right_trigger = gamepad2.right_trigger;
            g2_left_trigger = gamepad2.left_trigger;
        }
                //Setting boolean hold
        if(g1_right_bumper) {
            //Slowmode
            finalSlowmode = slowSpeed;

        } else if (g1_left_bumper) {
            //Fastmode
            finalSlowmode = fastSpeed;
        } else {
            //Regular
            finalSlowmode = driveSpeed;
        }

        if (g1_y){
            imu.resetYaw();
        }


        orientation = imu.getRobotYawPitchRollAngles();
        mecanumDrive.driveFieldCentric(g1_leftstick_x * finalSlowmode, -g1_leftstick_y * finalSlowmode, gamepad1.right_stick_x * finalSlowmode, orientation.getYaw(AngleUnit.DEGREES));
    }
}
