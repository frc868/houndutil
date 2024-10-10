package com.techhounds.houndutil.houndlib.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * Handles input from a Virpil Controls VPC Alpha-R joystick.
 * 
 * @apiNote When the trigger is in the neutral position, neither
 *          {@code kFlipTriggerOut} nor {@code kFlipTriggerIn} will be active.
 */
public class VirpilJoystick extends GenericHID {
    /** Represents a digital button on a joystick. */
    public enum Button {
        /** Red button on the front face. */
        kRedButton(1),
        /** Black button on the front face. */
        kBlackThumbButton(2),
        /** Black button on the pinkie side of the joystick. */
        kPinkieButton(3),
        /** Dual-stage trigger first stage. */
        kTriggerSoftPress(4),
        /** Dual-stage trigger second stage. */
        kTriggerHardPress(5),
        /** Flip trigger locked out. */
        kFlipTriggerOut(6),
        /** Flip trigger locked in against dual-stage trigger. */
        kFlipTriggerIn(7),
        /** Joystick button. */
        kStick(8),
        /** Press "in" on the HAT on the bottom left of the joystick. */
        kBottomHat(9),
        /**
         * Press "in" on the HAT on the bottom left of the front face of the joystick.
         */
        kCenterBottomHat(10),
        /** Press "in" on the HAT on the top right of the front face of the joystick. */
        kCenterTopHat(11),
        /** Press "in" on the HAT on the top right of the joystick, around the edge. */
        kTopRightHat(12),
        /** Dual-stage dial first stage. */
        kDialSoftPress(13),
        /** Dual-stage dial second stage. */
        kDialHardPress(14);

        public final int value;

        Button(int value) {
            this.value = value;
        }
    }

    public enum Axis {
        /** The X axis (left-right), right positive. */
        kX(0),
        /** The X axis (forward-back), backward positive. */
        kY(1),
        /** The twist (z) axis, CW positive. */
        kTwist(2),
        /** The X axis of the small joystick. */
        kStickX(3),
        /** The Y axis of the small joystick. */
        kStickY(4),
        /** The brake lever. */
        kLever(5);

        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }

    public enum HAT {
        /** The HAT on the bottom left of the joystick. */
        kBottom(0),
        /**
         * The HAT on the bottom left of the front face of the joystick.
         */
        kCenterBottom(1),
        /** The HAT on the top right of the front face of the joystick. */
        kCenterTop(2),
        /** The HAT on the top right of the joystick, around the edge. */
        kTopRight(3);

        public final int value;

        HAT(int value) {
            this.value = value;
        }
    }

    /**
     * Constructs a new VirpilJoystick on the specified port.
     *
     * @param port The port index on the Driver Station that the joystick is plugged
     *             into.
     */
    public VirpilJoystick(final int port) {
        super(port);
    }

    /**
     * Get the value of the X axis.
     *
     * @return The X axis value.
     */
    public double getX() {
        return getRawAxis(Axis.kX.value);
    }

    /**
     * Get the value of the Y axis.
     *
     * @return The Y axis value.
     */
    public double getY() {
        return getRawAxis(Axis.kY.value);
    }

    /**
     * Get the value of the twist (Z) axis.
     *
     * @return The twist axis value.
     */
    public double getTwist() {
        return getRawAxis(Axis.kTwist.value);
    }

    /**
     * Get the value of the small joystick X axis.
     *
     * @return The small joystick X axis value.
     */
    public double getStickX() {
        return getRawAxis(Axis.kStickX.value);
    }

    /**
     * Get the value of the small joystick Y axis.
     *
     * @return The small joystick Y axis value.
     */
    public double getStickY() {
        return getRawAxis(Axis.kStickY.value);
    }

    /**
     * Get the value of the lever axis.
     *
     * @return The lever axis value.
     */
    public double getLever() {
        return getRawAxis(Axis.kLever.value);
    }

    /**
     * Get the state of the red button.
     *
     * @return true if the red button is pressed, false otherwise.
     */
    public boolean getRedButton() {
        return getRawButton(Button.kRedButton.value);
    }

    /**
     * Check if the red button was pressed since the last check.
     *
     * @return true if the red button was pressed since the last check, false
     *         otherwise.
     */
    public boolean getRedButtonPressed() {
        return getRawButtonPressed(Button.kRedButton.value);
    }

    /**
     * Check if the red button was released since the last check.
     *
     * @return true if the red button was released since the last check, false
     *         otherwise.
     */
    public boolean getRedButtonReleased() {
        return getRawButtonReleased(Button.kRedButton.value);
    }

    /**
     * Creates a BooleanEvent for the red button.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the red button.
     */
    public BooleanEvent redButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getRedButton);
    }

    /**
     * Get the state of the black thumb button.
     *
     * @return true if the black thumb button is pressed, false otherwise.
     */
    public boolean getBlackThumbButton() {
        return getRawButton(Button.kBlackThumbButton.value);
    }

    /**
     * Check if the black thumb button was pressed since the last check.
     *
     * @return true if the black thumb button was pressed since the last check,
     *         false otherwise.
     */
    public boolean getBlackThumbButtonPressed() {
        return getRawButtonPressed(Button.kBlackThumbButton.value);
    }

    /**
     * Check if the black thumb button was released since the last check.
     *
     * @return true if the black thumb button was released since the last check,
     *         false otherwise.
     */
    public boolean getBlackThumbButtonReleased() {
        return getRawButtonReleased(Button.kBlackThumbButton.value);
    }

    /**
     * Creates a BooleanEvent for the black thumb button.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the black thumb button.
     */
    public BooleanEvent blackThumbButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getBlackThumbButton);
    }

    /**
     * Get the state of the pinkie button.
     *
     * @return true if the pinkie button is pressed, false otherwise.
     */
    public boolean getPinkieButton() {
        return getRawButton(Button.kPinkieButton.value);
    }

    /**
     * Check if the pinkie button was pressed since the last check.
     *
     * @return true if the pinkie button was pressed since the last check, false
     *         otherwise.
     */
    public boolean getPinkieButtonPressed() {
        return getRawButtonPressed(Button.kPinkieButton.value);
    }

    /**
     * Check if the pinkie button was released since the last check.
     *
     * @return true if the pinkie button was released since the last check, false
     *         otherwise.
     */
    public boolean getPinkieButtonReleased() {
        return getRawButtonReleased(Button.kPinkieButton.value);
    }

    /**
     * Creates a BooleanEvent for the pinkie button.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the pinkie button.
     */
    public BooleanEvent pinkieButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getPinkieButton);
    }

    /**
     * Get the state of the trigger soft press.
     *
     * @return true if the trigger soft press is pressed, false otherwise.
     */
    public boolean getTriggerSoftPress() {
        return getRawButton(Button.kTriggerSoftPress.value);
    }

    /**
     * Check if the trigger soft press was pressed since the last check.
     *
     * @return true if the trigger soft press was pressed since the last check,
     *         false otherwise.
     */
    public boolean getTriggerSoftPressPressed() {
        return getRawButtonPressed(Button.kTriggerSoftPress.value);
    }

    /**
     * Check if the trigger soft press was released since the last check.
     *
     * @return true if the trigger soft press was released since the last check,
     *         false otherwise.
     */
    public boolean getTriggerSoftPressReleased() {
        return getRawButtonReleased(Button.kTriggerSoftPress.value);
    }

    /**
     * Creates a BooleanEvent for the trigger soft press.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the trigger soft press.
     */
    public BooleanEvent triggerSoftPress(EventLoop loop) {
        return new BooleanEvent(loop, this::getTriggerSoftPress);
    }

    /**
     * Get the state of the trigger hard press.
     *
     * @return true if the trigger hard press is pressed, false otherwise.
     */
    public boolean getTriggerHardPress() {
        return getRawButton(Button.kTriggerHardPress.value);
    }

    /**
     * Check if the trigger hard press was pressed since the last check.
     *
     * @return true if the trigger hard press was pressed since the last check,
     *         false otherwise.
     */
    public boolean getTriggerHardPressPressed() {
        return getRawButtonPressed(Button.kTriggerHardPress.value);
    }

    /**
     * Check if the trigger hard press was released since the last check.
     *
     * @return true if the trigger hard press was released since the last check,
     *         false otherwise.
     */
    public boolean getTriggerHardPressReleased() {
        return getRawButtonReleased(Button.kTriggerHardPress.value);
    }

    /**
     * Creates a BooleanEvent for the trigger hard press.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the trigger hard press.
     */
    public BooleanEvent triggerHardPress(EventLoop loop) {
        return new BooleanEvent(loop, this::getTriggerHardPress);
    }

    /**
     * Get the state of the flip trigger out.
     *
     * @return true if the flip trigger out is pressed, false otherwise.
     */
    public boolean getFlipTriggerOut() {
        return getRawButton(Button.kFlipTriggerOut.value);
    }

    /**
     * Check if the flip trigger out was pressed since the last check.
     *
     * @return true if the flip trigger out was pressed since the last check, false
     *         otherwise.
     */
    public boolean getFlipTriggerOutPressed() {
        return getRawButtonPressed(Button.kFlipTriggerOut.value);
    }

    /**
     * Check if the flip trigger out was released since the last check.
     *
     * @return true if the flip trigger out was released since the last check, false
     *         otherwise.
     */
    public boolean getFlipTriggerOutReleased() {
        return getRawButtonReleased(Button.kFlipTriggerOut.value);
    }

    /**
     * Creates a BooleanEvent for the flip trigger out.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the flip trigger out.
     */
    public BooleanEvent flipTriggerOut(EventLoop loop) {
        return new BooleanEvent(loop, this::getFlipTriggerOut);
    }

    /**
     * Get the state of the flip trigger in.
     *
     * @return true if the flip trigger in is pressed, false otherwise.
     */
    public boolean getFlipTriggerIn() {
        return getRawButton(Button.kFlipTriggerIn.value);
    }

    /**
     * Check if the flip trigger in was pressed since the last check.
     *
     * @return true if the flip trigger in was pressed since the last check, false
     *         otherwise.
     */
    public boolean getFlipTriggerInPressed() {
        return getRawButtonPressed(Button.kFlipTriggerIn.value);
    }

    /**
     * Check if the flip trigger in was released since the last check.
     *
     * @return true if the flip trigger in was released since the last check, false
     *         otherwise.
     */
    public boolean getFlipTriggerInReleased() {
        return getRawButtonReleased(Button.kFlipTriggerIn.value);
    }

    /**
     * Creates a BooleanEvent for the flip trigger in.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the flip trigger in.
     */
    public BooleanEvent flipTriggerIn(EventLoop loop) {
        return new BooleanEvent(loop, this::getFlipTriggerIn);
    }

    /**
     * Get the state of the stick button.
     *
     * @return true if the stick button is pressed, false otherwise.
     */
    public boolean getStickButton() {
        return getRawButton(Button.kStick.value);
    }

    /**
     * Check if the stick button was pressed since the last check.
     *
     * @return true if the stick button was pressed since the last check, false
     *         otherwise.
     */
    public boolean getStickButtonPressed() {
        return getRawButtonPressed(Button.kStick.value);
    }

    /**
     * Check if the stick button was released since the last check.
     *
     * @return true if the stick button was released since the last check, false
     *         otherwise.
     */
    public boolean getStickButtonReleased() {
        return getRawButtonReleased(Button.kStick.value);
    }

    /**
     * Creates a BooleanEvent for the stick button.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the stick button.
     */
    public BooleanEvent stickButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getStickButton);
    }

    /**
     * Get the state of the bottom hat button.
     *
     * @return true if the bottom hat button is pressed, false otherwise.
     */
    public boolean getBottomHatButton() {
        return getRawButton(Button.kBottomHat.value);
    }

    /**
     * Check if the bottom hat button was pressed since the last check.
     *
     * @return true if the bottom hat button was pressed since the last check, false
     *         otherwise.
     */
    public boolean getBottomHatButtonPressed() {
        return getRawButtonPressed(Button.kBottomHat.value);
    }

    /**
     * Check if the bottom hat button was released since the last check.
     *
     * @return true if the bottom hat button was released since the last check,
     *         false otherwise.
     */
    public boolean getBottomHatButtonReleased() {
        return getRawButtonReleased(Button.kBottomHat.value);
    }

    /**
     * Creates a BooleanEvent for the bottom hat button.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the bottom hat button.
     */
    public BooleanEvent bottomHatButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getBottomHatButton);
    }

    /**
     * Get the state of the center bottom hat button.
     *
     * @return true if the center bottom hat button is pressed, false otherwise.
     */
    public boolean getCenterBottomHatButton() {
        return getRawButton(Button.kCenterBottomHat.value);
    }

    /**
     * Check if the center bottom hat button was pressed since the last check.
     *
     * @return true if the center bottom hat button was pressed since the last
     *         check, false otherwise.
     */
    public boolean getCenterBottomHatButtonPressed() {
        return getRawButtonPressed(Button.kCenterBottomHat.value);
    }

    /**
     * Check if the center bottom hat button was released since the last check.
     *
     * @return true if the center bottom hat button was released since the last
     *         check, false otherwise.
     */
    public boolean getCenterBottomHatButtonReleased() {
        return getRawButtonReleased(Button.kCenterBottomHat.value);
    }

    /**
     * Creates a BooleanEvent for the center bottom hat button.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the center bottom hat button.
     */
    public BooleanEvent centerBottomHatButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getCenterBottomHatButton);
    }

    /**
     * Get the state of the center top hat button.
     *
     * @return true if the center top hat button is pressed, false otherwise.
     */
    public boolean getCenterTopHatButton() {
        return getRawButton(Button.kCenterTopHat.value);
    }

    /**
     * Check if the center top hat button was pressed since the last check.
     *
     * @return true if the center top hat button was pressed since the last check,
     *         false otherwise.
     */
    public boolean getCenterTopHatButtonPressed() {
        return getRawButtonPressed(Button.kCenterTopHat.value);
    }

    /**
     * Check if the center top hat button was released since the last check.
     *
     * @return true if the center top hat button was released since the last check,
     *         false otherwise.
     */
    public boolean getCenterTopHatButtonReleased() {
        return getRawButtonReleased(Button.kCenterTopHat.value);
    }

    /**
     * Creates a BooleanEvent for the center top hat button.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the center top hat button.
     */
    public BooleanEvent centerTopHatButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getCenterTopHatButton);
    }

    /**
     * Get the state of the top right hat button.
     *
     * @return true if the top right hat button is pressed, false otherwise.
     */
    public boolean getTopRightHatButton() {
        return getRawButton(Button.kTopRightHat.value);
    }

    /**
     * Check if the top right hat button was pressed since the last check.
     *
     * @return true if the top right hat button was pressed since the last check,
     *         false otherwise.
     */
    public boolean getTopRightHatButtonPressed() {
        return getRawButtonPressed(Button.kTopRightHat.value);
    }

    /**
     * Check if the top right hat button was released since the last check.
     *
     * @return true if the top right hat button was released since the last check,
     *         false otherwise.
     */
    public boolean getTopRightHatButtonReleased() {
        return getRawButtonReleased(Button.kTopRightHat.value);
    }

    /**
     * Creates a BooleanEvent for the top right hat button.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the top right hat button.
     */
    public BooleanEvent topRightHatButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getTopRightHatButton);
    }

    /**
     * Get the state of the bottom hat POV.
     *
     * @return The bottom hat POV value.
     */
    public int getBottomHat() {
        return getPOV(HAT.kBottom.value);
    }

    /**
     * Get the state of the center bottom hat POV.
     *
     * @return The center bottom hat POV value.
     */
    public int getCenterBottomHat() {
        return getPOV(HAT.kCenterBottom.value);
    }

    /**
     * Get the state of the center top hat POV.
     *
     * @return The center top hat POV value.
     */
    public int getCenterTopHat() {
        return getPOV(HAT.kCenterTop.value);
    }

    /**
     * Get the state of the top right hat POV.
     *
     * @return The top right hat POV value.
     */
    public int getTopRightHat() {
        return getPOV(HAT.kTopRight.value);
    }

    /**
     * Get the state of the dial soft press.
     *
     * @return true if the dial soft press is pressed, false otherwise.
     */
    public boolean getDialSoftPress() {
        return getRawButton(Button.kDialSoftPress.value);
    }

    /**
     * Check if the dial soft press was pressed since the last check.
     *
     * @return true if the dial soft press was pressed since the last check, false
     *         otherwise.
     */
    public boolean getDialSoftPressPressed() {
        return getRawButtonPressed(Button.kDialSoftPress.value);
    }

    /**
     * Check if the dial soft press was released since the last check.
     *
     * @return true if the dial soft press was released since the last check, false
     *         otherwise.
     */
    public boolean getDialSoftPressReleased() {
        return getRawButtonReleased(Button.kDialSoftPress.value);
    }

    /**
     * Creates a BooleanEvent for the dial soft press.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the dial soft press.
     */
    public BooleanEvent dialSoftPress(EventLoop loop) {
        return new BooleanEvent(loop, this::getDialSoftPress);
    }

    /**
     * Get the state of the dial hard press.
     *
     * @return true if the dial hard press is pressed, false otherwise.
     */
    public boolean getDialHardPress() {
        return getRawButton(Button.kDialHardPress.value);
    }

    /**
     * Check if the dial hard press was pressed since the last check.
     *
     * @return true if the dial hard press was pressed since the last check, false
     *         otherwise.
     */
    public boolean getDialHardPressPressed() {
        return getRawButtonPressed(Button.kDialHardPress.value);
    }

    /**
     * Check if the dial hard press was released since the last check.
     *
     * @return true if the dial hard press was released since the last check, false
     *         otherwise.
     */
    public boolean getDialHardPressReleased() {
        return getRawButtonReleased(Button.kDialHardPress.value);
    }

    /**
     * Creates a BooleanEvent for the dial hard press.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the dial hard press.
     */
    public BooleanEvent dialHardPress(EventLoop loop) {
        return new BooleanEvent(loop, this::getDialHardPress);
    }
}
