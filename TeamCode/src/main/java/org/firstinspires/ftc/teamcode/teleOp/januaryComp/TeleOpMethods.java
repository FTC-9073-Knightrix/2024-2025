package org.firstinspires.ftc.teamcode.teleOp.januaryComp;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public abstract class TeleOpMethods extends TeleOpHardwareMap {
    // ------------------------------------ TELEOP VARIABLES ------------------------------------
    //Drive train speeds
    final double driveSpeed = 0.66;
    final double fastSpeed = 1.0;
    final double slowSpeed = 0.25;
    double finalSlowMode = 0.0;

    //Servo variables
    double armDownPos = 0.68;
    double armDefaultPos = 0.5;
    double armAtBasketPos = 0.05;

    double armRot = armDefaultPos; // start the arm halfway
    double basketRot = 0.0;
    double latchRot = 1.0;
    double clawRot = 0.2;

    //Motor variables
    int liftPosHoriz = 0;
    int liftPosAdjHoriz = 0;
    int maximumHorizExtend; // TODO add when magnet sensor mounted
    double horizLinearPower = 0.0;

    int liftPosVert = 0;
    int liftPosAdjVert = 0;
    int maximumVertExtend = 3600;
    double vertLinearPower = 0.0;

    double intakePower = 0.0;

    double hangerPower = 0.0;

    // Gamepad Conditionals
    boolean clickedA = false;
    boolean startHanging = false;
    boolean latchManualToggle = false;
    boolean clawOpen = true;

    boolean robotCentric = false;

    //Automated time variables

    double autoIntakeCurrent1 = Double.MAX_VALUE;
    double autoIntakeCurrent2 = Double.MAX_VALUE;
    double autoIntakeCurrent3 = Double.MAX_VALUE;

    double basketDropCurrent1 = Double.MAX_VALUE;
    double basketDropCurrent2 = Double.MAX_VALUE;
    double basketDropCurrent3 = Double.MAX_VALUE;
    double basketDropCurrent4 = Double.MAX_VALUE;

    double latchCurrent1 =  Double.MAX_VALUE;

    public void runMecanumDrive(){
        // ---------------------------------------MECANUM DRIVE ---------------------------------------
        //Setting boolean hold
        if(gamepad1.right_bumper) {
            //Slowmode
            finalSlowMode = slowSpeed;

        } else if (gamepad1.left_bumper) {
            //Fastmode
            finalSlowMode = fastSpeed;
        } else {
            //Regular
            finalSlowMode = driveSpeed;
        }


        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * .8;

        if (gamepad1.y) {
            imu.resetYaw();
        }

        orientation = imu.getRobotYawPitchRollAngles();
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.set(frontLeftPower * finalSlowMode);
        leftBack.set(backLeftPower * finalSlowMode);
        rightFront.set(frontRightPower * finalSlowMode);
        rightBack.set(backRightPower * finalSlowMode);

        robotCentric = false;
    }

    @SuppressLint("DefaultLocale")
    public void addTelemetryToDriverStation() {
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("g2LStickY, g2RStickY", gamepad2.left_stick_y + ", " + gamepad2.right_stick_y );
        telemetry.addData("Vertical, True", liftPosVert + ", " + vertLinearMotor.getCurrentPosition());
        telemetry.addData("Horizontal, True", liftPosHoriz + ", " + horizLinearMotor.getCurrentPosition());
        telemetry.addData("Gyro: ", "Yaw: " + String.format("%.2f", orientation.getYaw(AngleUnit.DEGREES))
                + "Roll: " + String.format("%.2f", orientation.getRoll(AngleUnit.DEGREES))
                + "Pitch: " + String.format("%.2f", orientation.getPitch(AngleUnit.DEGREES)));
        telemetry.addData("Slowmode: ", finalSlowMode);
        telemetry.update();
    }
}
