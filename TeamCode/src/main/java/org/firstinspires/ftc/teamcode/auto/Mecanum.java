package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Mecanum {
    public final double maxVelocity = 12; // in/s

    private final DcMotorEx motorEsquerdaFrente;
    private final DcMotorEx motorDireitaFrente;
    private final DcMotorEx motorEsquerdaTras;
    private final DcMotorEx motorDireitaTras;

    public Mecanum(OpMode opMode) {
        motorEsquerdaFrente = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.motorEsquerdaFrente);
        motorDireitaFrente = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.motorDireitaFrente);
        motorEsquerdaTras = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.motorEsquerdaTras);
        motorDireitaTras = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.motorDireitaTras);

        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDrivetrainMode(DcMotor.RunMode modo)
    {
        DcMotor[] motores = {motorEsquerdaFrente, motorDireitaFrente, motorDireitaTras, motorEsquerdaTras};
        for(DcMotor motor : motores)
        {
            motor.setMode(modo);
        }
    }

    public void drive(double gamepadLY, double gamepadLX, double gamepadRX)
    {
        drive(gamepadLY, gamepadLX, gamepadRX, false);
    }

    public void drive(double gamepadLY, double gamepadLX, double gamepadRX, boolean fieldOriented)
    {
        double denominator = Math.max(Math.abs(gamepadLY) + Math.abs(gamepadLX) + Math.abs(gamepadRX), 1);
        double frontLeftPower = (gamepadLY + gamepadLX + gamepadRX) / denominator;
        double backLeftPower = (gamepadLY - gamepadLX + gamepadRX) / denominator;
        double frontRightPower = (gamepadLY - gamepadLX - gamepadRX) / denominator;
        double backRightPower = (gamepadLY + gamepadLX - gamepadRX) / denominator;
    }
}
