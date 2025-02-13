package org.firstinspires.ftc.teamcode.stemos;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.stemos.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.stemos.subsystem.IntakeSubsystem;


@Autonomous(name="Autonomous with Actionns", group="Iterative OpMode")
public class AutonomousWithActions extends OpMode
{

    ArmSubsystem arm;
    IntakeSubsystem intake;

    @Override
    public void init() {
        arm = new ArmSubsystem(this);
        intake = new IntakeSubsystem(this);

    }
    @Override
    public void start()
    {
        Actions.runBlocking(
                new SequentialAction(
                        arm.setArmAngle(360),
                        new SleepAction(0.5),
                        intake.closeIntakeAction(),
                        arm.setArmAngle(720),
                        new SleepAction(0.5),
                        intake.openIntakeAction(),
                        arm.setArmAngle(0)));
    }

    @Override
    public void loop() {

    }
}
