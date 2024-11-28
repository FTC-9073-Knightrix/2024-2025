package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Test")
public class testing11272024 extends AutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        ArmServo armServo = new ArmServo(hardwareMap);
        ClawServo clawServo = new ClawServo(hardwareMap);
        VertLift vertLift = new VertLift(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {
            intakeMotor.intakeIn().run(null);  /* I cant use roadrunner for some reason on my laptop
            so someone has to make it   intakeMotor.intakeIn().run(telemetryPacket()); type of stuff */
            sleep(2000);

            armServo.closeArm().run(null);
            sleep(1000);

            intakeMotor.intakeStop().run(null);
            sleep(500);
            intakeMotor.intakeOut().run(null);
            sleep(2000);

            clawServo.openClaw().run(null);
            sleep(1000);

            clawServo.closeClaw().run(null);
            sleep(2000);

            vertLift.vertLiftUp().run(null);
        }
    }
}
