package org.firstinspires.ftc.teamcode.util;

import java.util.function.BooleanSupplier;

public class ToggleButton {

    private final BooleanSupplier button;
    private boolean lastState = false;
    private boolean toggled = false;

    /**
     * @param button A BooleanSupplier, e.g. () -> gamepad1.b
     */
    public ToggleButton(BooleanSupplier button) {
        this.button = button;
    }

    /**
     * Call once per loop
     */
    public void update() {
        boolean current = button.getAsBoolean();;

        // rising edge detection
        if (current && !lastState) {
            toggled = !toggled;
        }

        lastState = current;
    }

    /**
     * @return current toggle state
     */
    public boolean isToggled() {
        return toggled;
    }

    /**
     * Optional: force state
     */
    public void set(boolean value) {
        toggled = value;
    }
}
