package org.firstinspires.ftc.teamcode.stemos;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.stemos.commands.BangBang;
import org.firstinspires.ftc.teamcode.stemos.commands.DeadReckoning;
import org.firstinspires.ftc.teamcode.stemos.commands.PIDCommand;
import org.firstinspires.ftc.teamcode.stemos.commands.controller.Controller;
import org.firstinspires.ftc.teamcode.stemos.subsystem.ArmSubsystem;

@Autonomous(name = "Auto for commands", group ="Iterative OpMode")
public class AutoForCommands extends OpMode
{

    ArmSubsystem arm;
    Controller controle;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        arm = new ArmSubsystem(this);
        controle = new Controller(gamepad1);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        if(controle.a.wasJustPressed())
        {
            Actions.runBlocking(new PIDCommand(arm, 360));
        } else if (controle.b.wasJustPressed())
        {
            Actions.runBlocking(new PIDCommand(arm, 360, new PIDFCoefficients(10, 0, 0, 0.001)));
        } else if(controle.x.wasJustPressed())
        {
            Actions.runBlocking(new BangBang(arm, 360));
        } else if(controle.y.wasJustPressed())
        {
            Actions.runBlocking(new DeadReckoning(arm, 1500));
        }

        controle.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
