package org.firstinspires.ftc.teamcode.stemos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;

public class BangBang implements Action {

    private final double targetDistance;
    private final Mecanum drivetrain;
    private static final double MARGEM = 2;

    public BangBang(Mecanum drivetrain, double distance)
    {
            this.drivetrain = drivetrain;
            targetDistance = distance;

            drivetrain.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drivetrain.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        double actualDistance = drivetrain.getLinearDistanceOfOneMotor();
        double error = targetDistance - actualDistance;

        telemetryPacket.put("Error", error);
        telemetryPacket.put("Distance", actualDistance);
        telemetryPacket.put("Target Distance", targetDistance);

        if(Math.abs(error) != MARGEM) {
            if (error >= MARGEM) {
                drivetrain.drive(0.7, 0, 0);
            } else if (error <= -MARGEM) {
                drivetrain.drive(-0.7, 0, 0);
            } else {
                drivetrain.drive(0, 0, 0);
            }
            return true;
        } else {
            return false;
        }
    }
}
