package org.firstinspires.ftc.teamcode.stemos.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.stemos.Constants;

public class IntakeSubsystem {

    private final ServoImplEx servo;

    public IntakeSubsystem(OpMode opMode)
    {
        servo = opMode.hardwareMap.get(ServoImplEx.class, Constants.IntakeNames.INTAKE);
    }

    public void closeIntake()
    {
        servo.setPosition(1.0);
    }

    public void openIntake()
    {
        servo.setPosition(0.0);
    }

    public double getPosition()
    {
        return servo.getPosition();
    }

    public Action closeIntakeAction() {
        return new InstantAction(this::closeIntake);
    }

    public Action openIntakeAction() {
        return new InstantAction(this::openIntake);
    }
}
