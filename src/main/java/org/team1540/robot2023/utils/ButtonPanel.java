package org.team1540.robot2023.utils;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonPanel extends GenericHID {

    public enum PanelButton {
        TOP_LEFT(1),
        TOP_CENTER(2),
        TOP_RIGHT(3),
        MIDDLE_LEFT(4),
        MIDDLE_CENTER(5),
        MIDDLE_RIGHT(6),
        BOTTOM_LEFT(7),
        BOTTOM_CENTER(8),
        BOTTOM_RIGHT(9);

        final int value;

        PanelButton(int value) {
            this.value = value;
        }
    }

    /**
     * Construct an instance of a device.
     *
     * @param port The port index on the Driver Station that the device is plugged into.
     */
    public ButtonPanel(int port) {
        super(port);
    }

    /**
     * Get the button value (starting at button 1).
     *
     * <p>The buttons are returned in a single 16 bit value with one bit representing the state of
     * each button. The appropriate button is returned as a boolean value.
     *
     * <p>This method returns true if the button is being held down at the time that this method is
     * being called.
     *
     * @param button The button to be read
     * @return The state of the button.
     */
    public boolean getButton(PanelButton button) {
        return this.getRawButton(button.value);
    }

    /**
     * Constructs an event instance around the button's digital signal.
     *
     * @param button The button to track
     * @return an event instance representing the button's digital signal attached to the {@link
     * CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger onButton(PanelButton button) {
        return this.onButton(button.value);
    }

    /**
     * Constructs an event instance around the button's digital signal.
     *
     * @param button The button number to track (starting at 1)
     * @return an event instance representing the button's digital signal attached to the {@link
     * CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger onButton(int button) {
        return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> this.getRawButton(button));
    }

    /**
     * Finds the first (closest to 0) button ID that is currently pressed
     * Returns -1 if no button is found
     *
     * @return The ID of the button
     */
    public int getPressed() {
        for (int i = 1; i <= 9; i++) {
            if (getRawButton(i)) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Returns true if any button is currently pressed
     *
     * @return true if any button on the panel is pressed
     */
    public boolean areAnyPressed() {
        return getPressed() != -1;
    }

    /**
     * Constructs an event instance around the collective digital signal of the buttons.
     *
     * @return an event instance representing if any of the buttons are pressed attached to the {@link
     * CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger onAny() {
        return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), this::areAnyPressed);
    }

}
