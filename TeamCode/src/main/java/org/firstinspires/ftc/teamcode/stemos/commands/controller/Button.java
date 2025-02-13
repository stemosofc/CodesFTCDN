package org.firstinspires.ftc.teamcode.stemos.commands.controller;

import androidx.annotation.NonNull;

/**
 * A Controller Button
 */
public class Button {
    private boolean isPressed;
    private boolean wasPressed;

    /**
     * Instantiates the Button
     */
    public Button() {}

    /**
     * The button's current state
     *
     * @return whether the button is pressed
     */
    public boolean isPressed() {
        return isPressed;
    }

    /**
     * The button's change of state
     *
     * @return whether the button was just pressed
     */
    public boolean wasJustPressed() {
        return isPressed && !wasPressed;
    }

    public boolean wasJustReleased() { return !isPressed && wasPressed; }

    /**
     * The Button as a String
     *
     * @return whether the Button is pressed
     */
    @NonNull
    public String toString() {
        return String.valueOf(isPressed);
    }

    /**
     * Updates the Button
     *
     * @param isNowPressed the new state
     */
    public void update(boolean isNowPressed) {
        wasPressed = isPressed;
        isPressed = isNowPressed;
    }
}