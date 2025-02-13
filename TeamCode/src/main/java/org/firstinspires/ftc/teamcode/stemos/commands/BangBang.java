package org.firstinspires.ftc.teamcode.stemos.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.stemos.subsystem.ArmSubsystem;

public class BangBang implements Action {

    private final double targetDistance;
    private final ArmSubsystem arm;
    private static final double MARGEM = 2;

    public BangBang(ArmSubsystem arm, double angle)
    {
        this.arm = arm;
        targetDistance = angle;

        arm.setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setArmMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        double actualDistance = arm.getAngleOfArm();
        double error = targetDistance - actualDistance;

        telemetryPacket.put("Error", error);
        telemetryPacket.put("Actual Pose", actualDistance);
        telemetryPacket.put("Target", targetDistance);

        if(Math.abs(error) != MARGEM) {
            if (error >= MARGEM) {
                arm.setSpeed(0.6);
            } else if (error <= -MARGEM) {
                arm.setSpeed(-0.6);
            } else {
                arm.setSpeed(0.0);
            }
            return true;
        } else {
            return false;
        }
    }
}
