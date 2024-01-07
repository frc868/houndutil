package com.techhounds.houndutil.houndlib.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class VirpilJoystick extends GenericHID {
    public enum Button {
        kRedButton(1),
        kBlackThumbButton(2),
        kPinkieButton(3),
        kTriggerSoftPress(4),
        kTriggerHardPress(5),
        kFlipTriggerOut(6),
        kFlipTriggerIn(7),
        kStick(8),
        kBottomHat(9),
        kCenterBottomHat(10),
        kCenterTopHat(11),
        kTopRightHat(12),
        kDialSoftPress(13),
        kDialHardPress(14);

        public final int value;

        Button(int value) {
            this.value = value;
        }
    }

    public enum Axis {
        kX(0),
        kY(1),
        kTwist(2),
        kStickX(3),
        kStickY(4),
        kLever(5);

        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }

    public enum HAT {
        kBottom(0),
        kCenterBottom(1),
        kCenterTop(2),
        kTopRight(3);

        public final int value;

        HAT(int value) {
            this.value = value;
        }
    }

    public VirpilJoystick(final int port) {
        super(port);
    }

    public double getX() {
        return getRawAxis(Axis.kX.value);
    }

    public double getY() {
        return getRawAxis(Axis.kY.value);
    }

    public double getTwist() {
        return getRawAxis(Axis.kTwist.value);
    }

    public double getStickX() {
        return getRawAxis(Axis.kStickX.value);
    }

    public double getStickY() {
        return getRawAxis(Axis.kStickY.value);
    }

    public double getLever() {
        return getRawAxis(Axis.kLever.value);
    }

    //

    public boolean getRedButton() {
        return getRawButton(Button.kRedButton.value);
    }

    public boolean getRedButtonPressed() {
        return getRawButtonPressed(Button.kRedButton.value);
    }

    public boolean getRedButtonReleased() {
        return getRawButtonReleased(Button.kRedButton.value);
    }

    public BooleanEvent redButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getRedButton);
    }

    //

    public boolean getBlackThumbButton() {
        return getRawButton(Button.kBlackThumbButton.value);
    }

    public boolean getBlackThumbButtonPressed() {
        return getRawButtonPressed(Button.kBlackThumbButton.value);
    }

    public boolean getBlackThumbButtonReleased() {
        return getRawButtonReleased(Button.kBlackThumbButton.value);
    }

    public BooleanEvent blackThumbButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getBlackThumbButton);
    }

    //

    public boolean getPinkieButton() {
        return getRawButton(Button.kPinkieButton.value);
    }

    public boolean getPinkieButtonPressed() {
        return getRawButtonPressed(Button.kPinkieButton.value);
    }

    public boolean getPinkieButtonReleased() {
        return getRawButtonReleased(Button.kPinkieButton.value);
    }

    public BooleanEvent pinkieButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getPinkieButton);
    }

    //

    public boolean getTriggerSoftPress() {
        return getRawButton(Button.kTriggerSoftPress.value);
    }

    public boolean getTriggerSoftPressPressed() {
        return getRawButtonPressed(Button.kTriggerSoftPress.value);
    }

    public boolean getTriggerSoftPressReleased() {
        return getRawButtonReleased(Button.kTriggerSoftPress.value);
    }

    public BooleanEvent triggerSoftPress(EventLoop loop) {
        return new BooleanEvent(loop, this::getTriggerSoftPress);
    }

    //

    public boolean getTriggerHardPress() {
        return getRawButton(Button.kTriggerHardPress.value);
    }

    public boolean getTriggerHardPressPressed() {
        return getRawButtonPressed(Button.kTriggerHardPress.value);
    }

    public boolean getTriggerHardPressReleased() {
        return getRawButtonReleased(Button.kTriggerHardPress.value);
    }

    public BooleanEvent triggerHardPress(EventLoop loop) {
        return new BooleanEvent(loop, this::getTriggerHardPress);
    }

    //

    public boolean getFlipTriggerOut() {
        return getRawButton(Button.kFlipTriggerOut.value);
    }

    public boolean getFlipTriggerOutPressed() {
        return getRawButtonPressed(Button.kFlipTriggerOut.value);
    }

    public boolean getFlipTriggerOutReleased() {
        return getRawButtonReleased(Button.kFlipTriggerOut.value);
    }

    public BooleanEvent flipTriggerOut(EventLoop loop) {
        return new BooleanEvent(loop, this::getFlipTriggerOut);
    }

    //

    public boolean getFlipTriggerIn() {
        return getRawButton(Button.kFlipTriggerIn.value);
    }

    public boolean getFlipTriggerInPressed() {
        return getRawButtonPressed(Button.kFlipTriggerIn.value);
    }

    public boolean getFlipTriggerInReleased() {
        return getRawButtonReleased(Button.kFlipTriggerIn.value);
    }

    public BooleanEvent flipTriggerIn(EventLoop loop) {
        return new BooleanEvent(loop, this::getFlipTriggerIn);
    }

    //

    public boolean getStickButton() {
        return getRawButton(Button.kStick.value);
    }

    public boolean getStickButtonPressed() {
        return getRawButtonPressed(Button.kStick.value);
    }

    public boolean getStickButtonReleased() {
        return getRawButtonReleased(Button.kStick.value);
    }

    public BooleanEvent stickButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getStickButton);
    }

    //

    public boolean getBottomHatButton() {
        return getRawButton(Button.kBottomHat.value);
    }

    public boolean getBottomHatButtonPressed() {
        return getRawButtonPressed(Button.kBottomHat.value);
    }

    public boolean getBottomHatButtonReleased() {
        return getRawButtonReleased(Button.kBottomHat.value);
    }

    public BooleanEvent bottomHatButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getBottomHatButton);
    }

    //

    public boolean getCenterBottomHatButton() {
        return getRawButton(Button.kCenterBottomHat.value);
    }

    public boolean getCenterBottomHatButtonPressed() {
        return getRawButtonPressed(Button.kCenterBottomHat.value);
    }

    public boolean getCenterBottomHatButtonReleased() {
        return getRawButtonReleased(Button.kCenterBottomHat.value);
    }

    public BooleanEvent centerBottomHatButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getCenterBottomHatButton);
    }

    //

    public boolean getCenterTopHatButton() {
        return getRawButton(Button.kCenterTopHat.value);
    }

    public boolean getCenterTopHatButtonPressed() {
        return getRawButtonPressed(Button.kCenterTopHat.value);
    }

    public boolean getCenterTopHatButtonReleased() {
        return getRawButtonReleased(Button.kCenterTopHat.value);
    }

    public BooleanEvent centerTopHatButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getCenterTopHatButton);
    }

    //

    public boolean getTopRightHatButton() {
        return getRawButton(Button.kTopRightHat.value);
    }

    public boolean getTopRightHatButtonPressed() {
        return getRawButtonPressed(Button.kTopRightHat.value);
    }

    public boolean getTopRightHatButtonReleased() {
        return getRawButtonReleased(Button.kTopRightHat.value);
    }

    public BooleanEvent topRightHatButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getTopRightHatButton);
    }

    //

    public int getBottomHat() {
        return getPOV(HAT.kBottom.value);
    }

    public int getCenterBottomHat() {
        return getPOV(HAT.kCenterBottom.value);
    }

    public int getCenterTopHat() {
        return getPOV(HAT.kCenterTop.value);
    }

    public int getTopRightHat() {
        return getPOV(HAT.kTopRight.value);
    }

    //

    public boolean getDialSoftPress() {
        return getRawButton(Button.kDialSoftPress.value);
    }

    public boolean getDialSoftPressPressed() {
        return getRawButtonPressed(Button.kDialSoftPress.value);
    }

    public boolean getDialSoftPressReleased() {
        return getRawButtonReleased(Button.kDialSoftPress.value);
    }

    public BooleanEvent dialSoftPress(EventLoop loop) {
        return new BooleanEvent(loop, this::getDialSoftPress);
    }

    //

    public boolean getDialHardPress() {
        return getRawButton(Button.kDialHardPress.value);
    }

    public boolean getDialHardPressPressed() {
        return getRawButtonPressed(Button.kDialHardPress.value);
    }

    public boolean getDialHardPressReleased() {
        return getRawButtonReleased(Button.kDialHardPress.value);
    }

    public BooleanEvent dialHardPress(EventLoop loop) {
        return new BooleanEvent(loop, this::getDialHardPress);
    }
}
