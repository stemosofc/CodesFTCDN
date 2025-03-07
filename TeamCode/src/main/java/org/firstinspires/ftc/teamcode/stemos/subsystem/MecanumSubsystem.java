package org.firstinspires.ftc.teamcode.stemos.subsystem;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.stemos.Constants;

public class MecanumSubsystem {
    private final DcMotorEx motorEsquerdaFrente;
    private final DcMotorEx motorDireitaFrente;
    private final DcMotorEx motorEsquerdaTras;
    private final DcMotorEx motorDireitaTras;
    private final IMU imu;

    public MecanumSubsystem(OpMode opMode) {
        motorEsquerdaFrente = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.MOTOR_ESQUERDA_FRENTE);
        motorDireitaFrente = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.MOTOR_DIREITA_FRENTE);
        motorEsquerdaTras = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.MOTOR_ESQUERDA_TRAS);
        motorDireitaTras = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.MOTOR_DIREITA_TRAS);

        motorEsquerdaFrente.setDirection(DcMotorSimple.Direction.REVERSE);
        motorEsquerdaTras.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = opMode.hardwareMap.get(IMU.class, Constants.IMUNames.IMU_NAME);

        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void setDrivetrainMode(DcMotor.RunMode modo)
    {
        DcMotorEx[] motors = {motorEsquerdaFrente, motorDireitaFrente, motorDireitaTras, motorEsquerdaTras};
        for(DcMotorEx motor : motors)
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
        double forward = gamepadLY;
        double strafe = gamepadLX;
        double turn = gamepadRX;
        double heading = Math.toRadians(getHeading());
        if(fieldOriented)
        {
            double forwardUpdated = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
            double strafeUpdated = strafe * Math.cos(-heading) - forward * Math.sin(-heading);

            forward = forwardUpdated;
            strafe = strafeUpdated;
        }

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (forward + strafe + turn) / denominator;
        double backLeftPower = (forward - strafe + turn) / denominator;
        double frontRightPower = (forward - strafe - turn) / denominator;
        double backRightPower = (forward + strafe - turn) / denominator;

        motorEsquerdaFrente.setPower(frontLeftPower);
        motorDireitaFrente.setPower(frontRightPower);
        motorEsquerdaTras.setPower(backLeftPower);
        motorDireitaTras.setPower(backRightPower);
    }

    public double getHeading()
    {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading()
    {
        imu.resetYaw();
    }
}

