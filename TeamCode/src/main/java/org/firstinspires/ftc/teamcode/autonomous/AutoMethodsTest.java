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

                            new SleepAction(2.0),
                            intakeMotor.intakeStop(),
                            armServo.closeArm(),
                            new SleepAction(.75),
                            intakeMotor.intakeOut(),
                            new SleepAction(2.0),
                            armServo.openArm(),
                            new SleepAction(0.75),
                            latchServo.closeLatch(),
                            new SleepAction(0.75),
                            vertLift.vertLiftUp(),
                            new SleepAction(0.75),
                            basketServo.openBasket(),
                            new SleepAction(1.5),
                            latchServo.openLatch(),
                            new SleepAction(0.75),
                            basketServo.closeBasket(),
                            new SleepAction(1.5),
                            vertLift.vertLiftDown()
                    )
            );
        }
    }
}