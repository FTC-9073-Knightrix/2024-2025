package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "Right Blue Specimen")
public abstract class RightBlueSpecimen extends AutoMethods {
    @Override
    public void runOpMode() throws InterruptedException{
        initRobot();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -72, Math.toRadians(270)));
        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        ArmServo armServo = new ArmServo(hardwareMap);
        ClawServo clawServo = new ClawServo(hardwareMap);
        VertLift vertLift = new VertLift(hardwareMap);
        LatchServo latchServo = new LatchServo(hardwareMap);
        BasketServo basketServo = new BasketServo(hardwareMap);

        waitForStart();
        Actions.runBlocking(clawServo.closeClaw());

        if (opModeIsActive()){
            Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                    vertLift.vertLiftUp()
                )
            ));
        }
    }
}
