package com.techhounds.houndutil.houndlib.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * Handles input from a Virpil Controls VPC ALPHA Prime R joystick.
 */
public class VirpilAlphaPrime extends GenericHID {
    /** Represents a digital button on a joystick. */
    public enum Button {
        /** Flip trigger locked out. */
        kFlipTriggerOut(1),
        /** Flip trigger locked in center position */
        kFlipTriggerNeutral(2),
        /** Flip trigger locked in against dual-stage trigger. */
        kFlipTriggerIn(3),
        /** Dual-stage trigger first stage. */
        kTriggerSoftPress(4),
        /** Dual-stage trigger second stage. */
        kTriggerHardPress(5),
        /** Joystick button. */
        kStick(6),
        /** Black button on the left of the front face. */
        kBlackThumbButtonLeft(7),
        /** Press "in" on the HAT on the top right of the front face of the joystick. */
        kCenterTopHat(8),
        /** Press the HAT on the top right of the front face of the joystick up */
        kCenterTopHatUp(9),
        /** Press the HAT on the top right of the front face of the joystick left */
        kCenterTopHatLeft(10),
        /** Press the HAT on the top right of the front face of the joystick down */
        kCenterTopHatDown(11),
        /** Press the HAT on the top right of the front face of the joystick right */
        kCenterTopHatRight(12),
        /** Black button on the right of the front face. */
        kBlackThumbButtonRight(13),
        /** Press "in" on the HAT on the bottom left of the front face of the joystick. */
        kCenterBottomHat(14),
        /** Press the HAT on the bottom left of the front face of the joystick up */
        kCenterBottomHatUp(15),
        /** Press the HAT on the bottom left of the front face of the joystick left */
        kCenterBottomHatLeft(16),
        /** Press the HAT on the bottom left of the front face of the joystick down */
        kCenterBottomHatDown(17),
        /** Press the HAT on the bottom left of the front face of the joystick right */
        kCenterBottomHatRight(18),
        /** Dual-stage dial first stage. */
        kDialSoftPress(19),
        /** Dual-stage dial second stage. */
        kDialHardPress(20),
        /** Begin rolling the dial up */
        kDialUp(21),
        /** Begin rolling the dial down */
        kDialDown(22),
        /** Press "in" on the HAT on the bottom left of the joystick. */
        kBottomHat(23),
        /** Press the HAT on the bottom left of the joystick forward */
        kBottomHatUp(24),
        /** Press the HAT on the bottom left of the joystick left */
        kBottomHatLeft(25),
        /** Press the HAT on the bottom left of the joystick in reverse */
        kBottomHatDown(26),
        /** Press the HAT on the bottom left of the joystick right */
        kBottomHatRight(27),
        /** Press "in" on the HAT on the top right of the joystick, around the edge. */
        kTopRightHat(28),
        /** Press the HAT on the top right of the joystick, around the edge, up */
        kTopRightHatUp(29),
        /** Press the HAT on the top right of the joystick, around the edge, down */
        kTopRightHatDown(30),
        /** Black button on the pinkie side of the joystick. */
        kPinkieButton(31),
        /** Begin pulling in the lever */
        kActuateLever(32);

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

    /**
     * Constructs a new VirpilJoystick on the specified port.
     *
     * @param port The port index on the Driver Station that the joystick is plugged
     *             into.
     */
    public VirpilAlphaPrime(final int port) {
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
     * Get the state of the left black thumb button.
     *
     * @return true if the button is pressed, false otherwise.
     */
    public boolean getLeftBlackThumbButton() {
        return getRawButton(Button.kBlackThumbButtonLeft.value);
    }

    /**
     * Check if the left black thumb button was pressed since the last check.
     *
     * @return true if the button was pressed since the last check, false
     *         otherwise.
     */
    public boolean getLeftBlackThumbButtonPressed() {
        return getRawButtonPressed(Button.kBlackThumbButtonLeft.value);
    }

    /**
     * Check if the left black thumb button was released since the last check.
     *
     * @return true if the button was released since the last check, false
     *         otherwise.
     */
    public boolean getBlackThumbButtonLeftReleased() {
        return getRawButtonReleased(Button.kBlackThumbButtonLeft.value);
    }

    /**
     * Creates a BooleanEvent for the left black thumb button.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the button.
     */
    public BooleanEvent blackLeftThumbButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeftBlackThumbButton);
    }

    /**
     * Get the state of the right black thumb button.
     *
     * @return true if the black thumb button is pressed, false otherwise.
     */
    public boolean getRightBlackThumbButton() {
        return getRawButton(Button.kBlackThumbButtonRight.value);
    }

    /**
     * Check if the right black thumb button was pressed since the last check.
     *
     * @return true if the button was pressed since the last check,
     *         false otherwise.
     */
    public boolean getRightBlackThumbButtonPressed() {
        return getRawButtonPressed(Button.kBlackThumbButtonRight.value);
    }

    /**
     * Check if the right black thumb button was released since the last check.
     *
     * @return true if the button was released since the last check,
     *         false otherwise.
     */
    public boolean getRightBlackThumbButtonReleased() {
        return getRawButtonReleased(Button.kBlackThumbButtonRight.value);
    }

    /**
     * Creates a BooleanEvent for the right black thumb button.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the button.
     */
    public BooleanEvent blackRightThumbButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getRightBlackThumbButton);
    }

    /**
     * Get the state of the pinkie button.
     *
     * @return true if the button is pressed, false otherwise.
     */
    public boolean getPinkieButton() {
        return getRawButton(Button.kPinkieButton.value);
    }

    /**
     * Check if the pinkie button was pressed since the last check.
     *
     * @return true if the button was pressed since the last check, false
     *         otherwise.
     */
    public boolean getPinkieButtonPressed() {
        return getRawButtonPressed(Button.kPinkieButton.value);
    }

    /**
     * Check if the pinkie button was released since the last check.
     *
     * @return true if the button was released since the last check, false
     *         otherwise.
     */
    public boolean getPinkieButtonReleased() {
        return getRawButtonReleased(Button.kPinkieButton.value);
    }

    /**
     * Creates a BooleanEvent for the pinkie button.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the button.
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
     * Get the state of the flip trigger neutral.
     *
     * @return true if the flip trigger neutral is pressed, false otherwise.
     */
    public boolean getFlipTriggerNeutral() {
        return getRawButton(Button.kFlipTriggerNeutral.value);
    }

    /**
     * Check if the flip trigger neutral was pressed since the last check.
     *
     * @return true if the flip trigger neutral was pressed since the last check, false
     *         otherwise.
     */
    public boolean getFlipTriggerNeutralPressed() {
        return getRawButtonPressed(Button.kFlipTriggerNeutral.value);
    }

    /**
     * Check if the flip trigger neutral was released since the last check.
     *
     * @return true if the flip trigger neutral was released since the last check, false
     *         otherwise.
     */
    public boolean getFlipTriggerNeutralReleased() {
        return getRawButtonReleased(Button.kFlipTriggerNeutral.value);
    }

    /**
     * Creates a BooleanEvent for the flip trigger neutral.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the flip trigger neutral.
     */
    public BooleanEvent flipTriggerNeutral(EventLoop loop) {
        return new BooleanEvent(loop, this::getFlipTriggerNeutral);
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

    /**
     * Get the state of the center top hat up button.
     *
     * @return true if the center top hat up button is pressed, false otherwise.
     */
    public boolean getCenterTopHatUp() {
        return getRawButton(Button.kCenterTopHatUp.value);
    }

    /**
     * Check if the center top hat up button was pressed since the last check.
     *
     * @return true if the button was pressed since the last check, false otherwise.
     */
    public boolean getCenterTopHatUpPressed() {
        return getRawButtonPressed(Button.kCenterTopHatUp.value);
    }

    /**
     * Check if the center top hat up button was released since the last check.
     *
     * @return true if the button was released since the last check, false otherwise.
     */
    public boolean getCenterTopHatUpReleased() {
        return getRawButtonReleased(Button.kCenterTopHatUp.value);
    }

    /**
     * Creates a BooleanEvent for the center top hat up button.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the button.
     */
    public BooleanEvent centerTopHatUp(EventLoop loop) {
        return new BooleanEvent(loop, this::getCenterTopHatUp);
    }

    /**
     * Get the state of the dial up.
     *
     * @return true if the dial up is pressed, false otherwise.
     */
    public boolean getDialUp() {
        return getRawButton(Button.kDialUp.value);
    }

    /**
     * Check if the dial up was pressed since the last check.
     *
     * @return true if the dial up was pressed since the last check, false otherwise.
     */
    public boolean getDialUpPressed() {
        return getRawButtonPressed(Button.kDialUp.value);
    }

    /**
     * Check if the dial up was released since the last check.
     *
     * @return true if the dial up was released since the last check, false otherwise.
     */
    public boolean getDialUpReleased() {
        return getRawButtonReleased(Button.kDialUp.value);
    }

    /**
     * Creates a BooleanEvent for the dial up.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the dial up.
     */
    public BooleanEvent dialUp(EventLoop loop) {
        return new BooleanEvent(loop, this::getDialUp);
    }

    /**
     * Get the state of the dial down.
     *
     * @return true if the dial down is pressed, false otherwise.
     */
    public boolean getDialDown() {
        return getRawButton(Button.kDialDown.value);
    }

    /**
     * Check if the dial down was pressed since the last check.
     *
     * @return true if the dial down was pressed since the last check, false otherwise.
     */
    public boolean getDialDownPressed() {
        return getRawButtonPressed(Button.kDialDown.value);
    }

    /**
     * Check if the dial down was released since the last check.
     *
     * @return true if the dial down was released since the last check, false otherwise.
     */
    public boolean getDialDownReleased() {
        return getRawButtonReleased(Button.kDialDown.value);
    }

    /**
     * Creates a BooleanEvent for the dial down.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the dial down.
     */
    public BooleanEvent dialDown(EventLoop loop) {
        return new BooleanEvent(loop, this::getDialDown);
    }

    /**
     * Get the state of the center top hat left button.
     *
     * @return true if the center top hat left is pressed, false otherwise.
     */
    public boolean getCenterTopHatLeft() {
        return getRawButton(Button.kCenterTopHatLeft.value);
    }

    /**
     * Check if the center top hat left was pressed since the last check.
     *
     * @return true if the center top hat left was pressed since the last check, false otherwise.
     */
    public boolean getCenterTopHatLeftPressed() {
        return getRawButtonPressed(Button.kCenterTopHatLeft.value);
    }

    /**
     * Check if the center top hat left was released since the last check.
     *
     * @return true if the center top hat left was released since the last check, false otherwise.
     */
    public boolean getCenterTopHatLeftReleased() {
        return getRawButtonReleased(Button.kCenterTopHatLeft.value);
    }

    /**
     * Creates a BooleanEvent for the center top hat left.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the center top hat left.
     */
    public BooleanEvent centerTopHatLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getCenterTopHatLeft);
    }

    /**
     * Get the state of the center top hat down button.
     *
     * @return true if the center top hat down is pressed, false otherwise.
     */
    public boolean getCenterTopHatDown() {
        return getRawButton(Button.kCenterTopHatDown.value);
    }

    /**
     * Check if the center top hat down was pressed since the last check.
     *
     * @return true if the center top hat down was pressed since the last check, false otherwise.
     */
    public boolean getCenterTopHatDownPressed() {
        return getRawButtonPressed(Button.kCenterTopHatDown.value);
    }

    /**
     * Check if the center top hat down was released since the last check.
     *
     * @return true if the center top hat down was released since the last check, false otherwise.
     */
    public boolean getCenterTopHatDownReleased() {
        return getRawButtonReleased(Button.kCenterTopHatDown.value);
    }

    /**
     * Creates a BooleanEvent for the center top hat down.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the center top hat down.
     */
    public BooleanEvent centerTopHatDown(EventLoop loop) {
        return new BooleanEvent(loop, this::getCenterTopHatDown);
    }

    /**
     * Get the state of the center top hat right button.
     *
     * @return true if the center top hat right is pressed, false otherwise.
     */
    public boolean getCenterTopHatRight() {
        return getRawButton(Button.kCenterTopHatRight.value);
    }

    /**
     * Check if the center top hat right was pressed since the last check.
     *
     * @return true if the center top hat right was pressed since the last check, false otherwise.
     */
    public boolean getCenterTopHatRightPressed() {
        return getRawButtonPressed(Button.kCenterTopHatRight.value);
    }

    /**
     * Check if the center top hat right was released since the last check.
     *
     * @return true if the center top hat right was released since the last check, false otherwise.
     */
    public boolean getCenterTopHatRightReleased() {
        return getRawButtonReleased(Button.kCenterTopHatRight.value);
    }

    /**
     * Creates a BooleanEvent for the center top hat right.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the center top hat right.
     */
    public BooleanEvent centerTopHatRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getCenterTopHatRight);
    }

    /**
     * Get the state of the center bottom hat up button.
     *
     * @return true if the center bottom hat up is pressed, false otherwise.
     */
    public boolean getCenterBottomHatUp() {
        return getRawButton(Button.kCenterBottomHatUp.value);
    }

    /**
     * Check if the center bottom hat up was pressed since the last check.
     *
     * @return true if the center bottom hat up was pressed since the last check, false otherwise.
     */
    public boolean getCenterBottomHatUpPressed() {
        return getRawButtonPressed(Button.kCenterBottomHatUp.value);
    }

    /**
     * Check if the center bottom hat up was released since the last check.
     *
     * @return true if the center bottom hat up was released since the last check, false otherwise.
     */
    public boolean getCenterBottomHatUpReleased() {
        return getRawButtonReleased(Button.kCenterBottomHatUp.value);
    }

    /**
     * Creates a BooleanEvent for the center bottom hat up.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the center bottom hat up.
     */
    public BooleanEvent centerBottomHatUp(EventLoop loop) {
        return new BooleanEvent(loop, this::getCenterBottomHatUp);
    }

    /**
     * Get the state of the center bottom hat left button.
     *
     * @return true if the center bottom hat left is pressed, false otherwise.
     */
    public boolean getCenterBottomHatLeft() {
        return getRawButton(Button.kCenterBottomHatLeft.value);
    }

    /**
     * Check if the center bottom hat left was pressed since the last check.
     *
     * @return true if the center bottom hat left was pressed since the last check, false otherwise.
     */
    public boolean getCenterBottomHatLeftPressed() {
        return getRawButtonPressed(Button.kCenterBottomHatLeft.value);
    }

    /**
     * Check if the center bottom hat left was released since the last check.
     *
     * @return true if the center bottom hat left was released since the last check, false otherwise.
     */
    public boolean getCenterBottomHatLeftReleased() {
        return getRawButtonReleased(Button.kCenterBottomHatLeft.value);
    }

    /**
     * Creates a BooleanEvent for the center bottom hat left.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the center bottom hat left.
     */
    public BooleanEvent centerBottomHatLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getCenterBottomHatLeft);
    }

    /**
     * Get the state of the center bottom hat down button.
     *
     * @return true if the center bottom hat down is pressed, false otherwise.
     */
    public boolean getCenterBottomHatDown() {
        return getRawButton(Button.kCenterBottomHatDown.value);
    }

    /**
     * Check if the center bottom hat down was pressed since the last check.
     *
     * @return true if the center bottom hat down was pressed since the last check, false otherwise.
     */
    public boolean getCenterBottomHatDownPressed() {
        return getRawButtonPressed(Button.kCenterBottomHatDown.value);
    }

    /**
     * Check if the center bottom hat down was released since the last check.
     *
     * @return true if the center bottom hat down was released since the last check, false otherwise.
     */
    public boolean getCenterBottomHatDownReleased() {
        return getRawButtonReleased(Button.kCenterBottomHatDown.value);
    }

    /**
     * Creates a BooleanEvent for the center bottom hat down.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the center bottom hat down.
     */
    public BooleanEvent centerBottomHatDown(EventLoop loop) {
        return new BooleanEvent(loop, this::getCenterBottomHatDown);
    }

    /**
     * Get the state of the center bottom hat right button.
     *
     * @return true if the center bottom hat right is pressed, false otherwise.
     */
    public boolean getCenterBottomHatRight() {
        return getRawButton(Button.kCenterBottomHatRight.value);
    }

    /**
     * Check if the center bottom hat right was pressed since the last check.
     *
     * @return true if the center bottom hat right was pressed since the last check, false otherwise.
     */
    public boolean getCenterBottomHatRightPressed() {
        return getRawButtonPressed(Button.kCenterBottomHatRight.value);
    }

    /**
     * Check if the center bottom hat right was released since the last check.
     *
     * @return true if the center bottom hat right was released since the last check, false otherwise.
     */
    public boolean getCenterBottomHatRightReleased() {
        return getRawButtonReleased(Button.kCenterBottomHatRight.value);
    }

    /**
     * Creates a BooleanEvent for the center bottom hat right.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the center bottom hat right.
     */
    public BooleanEvent centerBottomHatRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getCenterBottomHatRight);
    }

    /**
     * Get the state of the bottom hat up button.
     *
     * @return true if the bottom hat up is pressed, false otherwise.
     */
    public boolean getBottomHatUp() {
        return getRawButton(Button.kBottomHatUp.value);
    }

    /**
     * Check if the bottom hat up was pressed since the last check.
     *
     * @return true if the bottom hat up was pressed since the last check, false otherwise.
     */
    public boolean getBottomHatUpPressed() {
        return getRawButtonPressed(Button.kBottomHatUp.value);
    }

    /**
     * Check if the bottom hat up was released since the last check.
     *
     * @return true if the bottom hat up was released since the last check, false otherwise.
     */
    public boolean getBottomHatUpReleased() {
        return getRawButtonReleased(Button.kBottomHatUp.value);
    }

    /**
     * Creates a BooleanEvent for the bottom hat up.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the bottom hat up.
     */
    public BooleanEvent bottomHatUp(EventLoop loop) {
        return new BooleanEvent(loop, this::getBottomHatUp);
    }

    /**
     * Get the state of the bottom hat left button.
     *
     * @return true if the bottom hat left is pressed, false otherwise.
     */
    public boolean getBottomHatLeft() {
        return getRawButton(Button.kBottomHatLeft.value);
    }

    /**
     * Check if the bottom hat left was pressed since the last check.
     *
     * @return true if the bottom hat left was pressed since the last check, false otherwise.
     */
    public boolean getBottomHatLeftPressed() {
        return getRawButtonPressed(Button.kBottomHatLeft.value);
    }

    /**
     * Check if the bottom hat left was released since the last check.
     *
     * @return true if the bottom hat left was released since the last check, false otherwise.
     */
    public boolean getBottomHatLeftReleased() {
        return getRawButtonReleased(Button.kBottomHatLeft.value);
    }

    /**
     * Creates a BooleanEvent for the bottom hat left.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the bottom hat left.
     */
    public BooleanEvent bottomHatLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getBottomHatLeft);
    }

    /**
     * Get the state of the bottom hat down button.
     *
     * @return true if the bottom hat down is pressed, false otherwise.
     */
    public boolean getBottomHatDown() {
        return getRawButton(Button.kBottomHatDown.value);
    }

    /**
     * Check if the bottom hat down was pressed since the last check.
     *
     * @return true if the bottom hat down was pressed since the last check, false otherwise.
     */
    public boolean getBottomHatDownPressed() {
        return getRawButtonPressed(Button.kBottomHatDown.value);
    }

    /**
     * Check if the bottom hat down was released since the last check.
     *
     * @return true if the bottom hat down was released since the last check, false otherwise.
     */
    public boolean getBottomHatDownReleased() {
        return getRawButtonReleased(Button.kBottomHatDown.value);
    }

    /**
     * Creates a BooleanEvent for the bottom hat down.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the bottom hat down.
     */
    public BooleanEvent bottomHatDown(EventLoop loop) {
        return new BooleanEvent(loop, this::getBottomHatDown);
    }

    /**
     * Get the state of the bottom hat right button.
     *
     * @return true if the bottom hat right is pressed, false otherwise.
     */
    public boolean getBottomHatRight() {
        return getRawButton(Button.kBottomHatRight.value);
    }

    /**
     * Check if the bottom hat right was pressed since the last check.
     *
     * @return true if the bottom hat right was pressed since the last check, false otherwise.
     */
    public boolean getBottomHatRightPressed() {
        return getRawButtonPressed(Button.kBottomHatRight.value);
    }

    /**
     * Check if the bottom hat right was released since the last check.
     *
     * @return true if the bottom hat right was released since the last check, false otherwise.
     */
    public boolean getBottomHatRightReleased() {
        return getRawButtonReleased(Button.kBottomHatRight.value);
    }

    /**
     * Creates a BooleanEvent for the bottom hat right.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the bottom hat right.
     */
    public BooleanEvent bottomHatRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getBottomHatRight);
    }

    /**
     * Get the state of the top right hat up button.
     *
     * @return true if the top right hat up is pressed, false otherwise.
     */
    public boolean getTopRightHatUp() {
        return getRawButton(Button.kTopRightHatUp.value);
    }

    /**
     * Check if the top right hat up was pressed since the last check.
     *
     * @return true if the top right hat up was pressed since the last check, false otherwise.
     */
    public boolean getTopRightHatUpPressed() {
        return getRawButtonPressed(Button.kTopRightHatUp.value);
    }

    /**
     * Check if the top right hat up was released since the last check.
     *
     * @return true if the top right hat up was released since the last check, false otherwise.
     */
    public boolean getTopRightHatUpReleased() {
        return getRawButtonReleased(Button.kTopRightHatUp.value);
    }

    /**
     * Creates a BooleanEvent for the top right hat up.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the top right hat up.
     */
    public BooleanEvent topRightHatUp(EventLoop loop) {
        return new BooleanEvent(loop, this::getTopRightHatUp);
    }

    /**
     * Get the state of the top right hat down button.
     *
     * @return true if the top right hat down is pressed, false otherwise.
     */
    public boolean getTopRightHatDown() {
        return getRawButton(Button.kTopRightHatDown.value);
    }

    /**
     * Check if the top right hat down was pressed since the last check.
     *
     * @return true if the top right hat down was pressed since the last check, false otherwise.
     */
    public boolean getTopRightHatDownPressed() {
        return getRawButtonPressed(Button.kTopRightHatDown.value);
    }

    /**
     * Check if the top right hat down was released since the last check.
     *
     * @return true if the top right hat down was released since the last check, false otherwise.
     */
    public boolean getTopRightHatDownReleased() {
        return getRawButtonReleased(Button.kTopRightHatDown.value);
    }

    /**
     * Creates a BooleanEvent for the top right hat down.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the top right hat down.
     */
    public BooleanEvent topRightHatDown(EventLoop loop) {
        return new BooleanEvent(loop, this::getTopRightHatDown);
    }

    /**
     * Get the state of the actuate lever button.
     *
     * @return true if the actuate lever is pressed, false otherwise.
     */
    public boolean getActuateLever() {
        return getRawButton(Button.kActuateLever.value);
    }

    /**
     * Check if the actuate lever was pressed since the last check.
     *
     * @return true if the actuate lever was pressed since the last check, false otherwise.
     */
    public boolean getActuateLeverPressed() {
        return getRawButtonPressed(Button.kActuateLever.value);
    }

    /**
     * Check if the actuate lever was released since the last check.
     *
     * @return true if the actuate lever was released since the last check, false otherwise.
     */
    public boolean getActuateLeverReleased() {
        return getRawButtonReleased(Button.kActuateLever.value);
    }

    /**
     * Creates a BooleanEvent for the actuate lever.
     *
     * @param loop the event loop to attach the event to.
     * @return a BooleanEvent for the actuate lever.
     */
    public BooleanEvent actuateLever(EventLoop loop) {
        return new BooleanEvent(loop, this::getActuateLever);
    }
}
