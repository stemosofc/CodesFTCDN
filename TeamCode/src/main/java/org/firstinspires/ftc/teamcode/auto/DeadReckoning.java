package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.util.ElapsedTime;

public class DeadReckoning {
    private final double maxTime;
    private final double distance;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean initialized = false;
    private final Mecanum drivetrain;

    public DeadReckoning(Mecanum drivetrain, double distance)
    {
        this.drivetrain = drivetrain;
        this.distance = distance;
        maxTime = distance / drivetrain.maxVelocity;
    }

    public void run()
    {
        if(timer.seconds() <= maxTime)
        {
            drivetrain.drive(1, 0, 0);
        } else drivetrain.drive(0, 0, 0);
    }
}
