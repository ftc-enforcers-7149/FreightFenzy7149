package org.firstinspires.ftc.teamcode.Subsystems;

/**
 * Sets values using a user-defined method, only when the value changes
 * @param <T> The type of value to set
 */
public abstract class ValueSetter<T> {

    private T value, lastValue; //Writeable values
    private boolean disabled; //If the writing is paused

    /**
     * Updates the written value only when it has changed
     * Should be called at the beginning of every loop
     */
    public void update() {
        if (!disabled && value != lastValue) {
            writeValue(value);
            lastValue = value;
        }
    }

    /**
     * Actual code used to make hardware call, not to be used by user
     * @param value The value to use in the call
     */
    public abstract void writeValue(T value);

    /**
     * Set the value, to be called by user
     * @param value A value to write with
     */
    public void setValue(T value) {
        this.value = value;
    }

    /**
     * Disables the update function
     */
    public void disable() {
        disabled = true;
    }

    /**
     * Enables the update function
     */
    public void enable() {
        disabled = false;
    }
}
