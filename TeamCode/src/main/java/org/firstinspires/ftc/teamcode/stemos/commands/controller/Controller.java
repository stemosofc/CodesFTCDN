package org.firstinspires.ftc.teamcode.stemos.commands.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * A Controller
 */
public class Controller {
    private final Gamepad gamepad;
    public final Button leftBumper;
    public final Button rightBumper;
    public final Button dpadUp;
    public final Button dpadDown;
    public final Button dpadLeft;
    public final Button dpadRight;
    public final Button a;
    public final Button b;
    public final Button x;
    public final Button y;
    public final Button share;
    public final Button options;
    public final Button leftStickButton;
    public final Button rightStickButton;

    /**
     * Instantiates the Controller
     *
     * @param gamepad the gamepad to use
     */
    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;

        leftBumper = new Button();
        rightBumper = new Button();
        dpadUp = new Button();
        dpadDown = new Button();
        dpadLeft = new Button();
        dpadRight = new Button();
        x = new Button();
        b = new Button();
        a = new Button();
        y = new Button();
        share = new Button();
        options = new Button();
        leftStickButton = new Button();
        rightStickButton = new Button();
    }

    /**
     * Updates all Buttons, Axes, and Joysticks
     */
    public void update() {
        leftBumper.update(gamepad.left_bumper);
        rightBumper.update(gamepad.right_bumper);
        dpadUp.update(gamepad.dpad_up);
        dpadDown.update(gamepad.dpad_down);
        dpadLeft.update(gamepad.dpad_left);
        dpadRight.update(gamepad.dpad_right);
        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);
        share.update(gamepad.back);
        options.update(gamepad.start);
        leftStickButton.update(gamepad.left_stick_button);
        rightStickButton.update(gamepad.right_stick_button);
    }
}