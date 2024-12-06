package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Test")
public class AutoMethodsTest extends AutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        ArmServo armServo = new ArmServo(hardwareMap);
        ClawServo clawServo = new ClawServo(hardwareMap);
        VertLift vertLift = new VertLift(hardwareMap);
        LatchServo latchServo = new LatchServo(hardwareMap);
        BasketServo basketServo = new BasketServo(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            latchServo.openLatch(),
                            intakeMotor.intakeIn(),
                            intakeMotor.intakeStop(),
                            armServo.closeArm(),
                            intakeMotor.intakeOut(),
                            armServo.openArm()
                    )
            );
        }
    }
}