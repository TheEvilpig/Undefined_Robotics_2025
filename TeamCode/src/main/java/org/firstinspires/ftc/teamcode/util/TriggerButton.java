package org.firstinspires.ftc.teamcode.util;

import java.util.function.DoubleSupplier;

public class TriggerButton {

    private final DoubleSupplier trigger;
    private final double threshold;

    /**
     * @param trigger   e.g. () -> gamepad1.right_trigger
     * @param threshold value above which the trigger is considered "pressed"
     */
    public TriggerButton(DoubleSupplier trigger, double threshold) {
        this.trigger = trigger;
        this.threshold = threshold;
    }

    /**
     * @return true while trigger is pressed
     */
    public boolean isPressed() {
        return trigger.getAsDouble() >= threshold;
    }
}
