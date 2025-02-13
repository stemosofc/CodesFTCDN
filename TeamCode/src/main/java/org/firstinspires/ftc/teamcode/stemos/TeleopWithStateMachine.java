package org.firstinspires.ftc.teamcode.stemos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.stemos.commands.controller.Controller;
import org.firstinspires.ftc.teamcode.stemos.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.stemos.subsystem.IntakeSubsystem;

@TeleOp(name="Teleop with State Machine", group="Iterative OpMode")
public class TeleopWithStateMachine extends OpMode {

    ArmSubsystem arm;
    IntakeSubsystem intake;
    Controller controle;

    private enum ArmState
    {
        INITIAL_POSE,
        EXTRACT_POSE,
        FINAL_POSE,
        RETRACT_POSE
    }

    ArmState armState = ArmState.INITIAL_POSE;

    boolean intakeToggle = false;

    @Override
    public void init() {
        arm = new ArmSubsystem(this);
        intake = new IntakeSubsystem(this);
        controle = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        switch(armState)
        {
            case INITIAL_POSE:
                if(controle.a.wasJustPressed())
                {
                    arm.setAngleOfArm(30);
                    armState = ArmState.EXTRACT_POSE;
                }
                break;
            case EXTRACT_POSE:
            {
                if(controle.a.wasJustPressed())
                {
                    arm.setAngleOfArm(180);
                    armState = ArmState.FINAL_POSE;
                }
                if(arm.atTarget())
                {
                    if(controle.x.wasJustPressed())
                    {
                        intakeToggle = !intakeToggle;
                        if(intakeToggle)
                        {
                            intake.openIntake();
                        } else
                        {
                            intake.closeIntake();
                        }
                    }
                }
                break;
            }
            case FINAL_POSE:
            {
                if(controle.a.wasJustPressed())
                {
                    arm.setAngleOfArm(0);
                    armState = ArmState.RETRACT_POSE;
                }
                if(arm.atTarget())
                {
                    if(controle.x.wasJustPressed())
                    {
                        intake.openIntake();
                    }
                }
                break;
            }
            case RETRACT_POSE:
            {
                if(Math.abs(arm.getAngleOfArm()) < 5)
                {
                    armState = ArmState.INITIAL_POSE;
                }
                break;
            }
            default:
                armState = ArmState.INITIAL_POSE;
        }

        if(controle.y.wasJustPressed()) {
            armState = ArmState.INITIAL_POSE;
            arm.setAngleOfArm(0);
        }

        controle.update();

        // Mecanum drive aqui!
        telemetry.addData("Arm State", armState);
        telemetry.addData("Arm position", arm.getAngleOfArm());
        telemetry.addData("Intake position", intake.getPosition());
        telemetry.update();
    }

}
