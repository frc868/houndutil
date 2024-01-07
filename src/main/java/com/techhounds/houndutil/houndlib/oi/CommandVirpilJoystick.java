package com.techhounds.houndutil.houndlib.oi;

import com.techhounds.houndutil.houndlib.oi.VirpilJoystick.HAT;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandVirpilJoystick extends CommandGenericHID {
    private final VirpilJoystick joystick;

    public CommandVirpilJoystick(int port) {
        super(port);

        joystick = new VirpilJoystick(port);
    }

    @Override
    public VirpilJoystick getHID() {
        return joystick;
    }

    public double getX() {
        return joystick.getX();
    }

    public double getY() {
        return joystick.getY();
    }

    public double getTwist() {
        return joystick.getTwist();
    }

    public double getStickX() {
        return joystick.getStickX();
    }

    public double getStickY() {
        return joystick.getStickY();
    }

    public double getLever() {
        return joystick.getLever();
    }

    public Trigger redButton() {
        return redButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger redButton(EventLoop loop) {
        return joystick.redButton(loop).castTo(Trigger::new);
    }

    public Trigger blackThumbButton() {
        return blackThumbButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger blackThumbButton(EventLoop loop) {
        return joystick.blackThumbButton(loop).castTo(Trigger::new);
    }

    public Trigger pinkieButton() {
        return pinkieButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger pinkieButton(EventLoop loop) {
        return joystick.triggerSoftPress(loop).castTo(Trigger::new);
    }

    public Trigger triggerSoftPress() {
        return triggerSoftPress(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger triggerSoftPress(EventLoop loop) {
        return joystick.triggerSoftPress(loop).castTo(Trigger::new);
    }

    public Trigger triggerHardPress() {
        return triggerHardPress(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger triggerHardPress(EventLoop loop) {
        return joystick.triggerHardPress(loop).castTo(Trigger::new);
    }

    public Trigger flipTriggerOut() {
        return flipTriggerOut(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger flipTriggerOut(EventLoop loop) {
        return joystick.flipTriggerOut(loop).castTo(Trigger::new);
    }

    public Trigger flipTriggerIn() {
        return flipTriggerIn(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger flipTriggerIn(EventLoop loop) {
        return joystick.flipTriggerIn(loop).castTo(Trigger::new);
    }

    public Trigger stickButton() {
        return stickButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger stickButton(EventLoop loop) {
        return joystick.stickButton(loop).castTo(Trigger::new);
    }

    public Trigger bottomHatButton() {
        return bottomHatButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger bottomHatButton(EventLoop loop) {
        return joystick.bottomHatButton(loop).castTo(Trigger::new);
    }

    public Trigger centerBottomHatButton() {
        return centerBottomHatButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger centerBottomHatButton(EventLoop loop) {
        return joystick.centerBottomHatButton(loop).castTo(Trigger::new);
    }

    public Trigger centerTopHatButton() {
        return centerTopHatButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger centerTopHatButton(EventLoop loop) {
        return joystick.centerTopHatButton(loop).castTo(Trigger::new);
    }

    public Trigger topRightHatButton() {
        return topRightHatButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger topRightHatButton(EventLoop loop) {
        return joystick.topRightHatButton(loop).castTo(Trigger::new);
    }

    public Trigger pov(int pov, int angle) {
        return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> joystick.getPOV(pov) == angle);
    }

    public Trigger pov(int pov, int angle, EventLoop loop) {
        return new Trigger(loop, () -> joystick.getPOV(pov) == angle);
    }

    public Trigger bottomHat(int angle) {
        return pov(HAT.kBottom.value, angle, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger centerBottomHat(int angle) {
        return pov(HAT.kCenterBottom.value, angle, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger centerTopHat(int angle) {
        return pov(HAT.kCenterTop.value, angle, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger topRightHat(int angle) {
        return pov(HAT.kTopRight.value, angle, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger bottomHatUp() {
        return bottomHat(0);
    }

    public Trigger bottomHatUpRight() {
        return bottomHat(45);
    }

    public Trigger bottomHatRight() {
        return bottomHat(90);
    }

    public Trigger bottomHatDownRight() {
        return bottomHat(135);
    }

    public Trigger bottomHatDown() {
        return bottomHat(180);
    }

    public Trigger bottomHatDownLeft() {
        return bottomHat(225);
    }

    public Trigger bottomHatLeft() {
        return bottomHat(270);
    }

    public Trigger bottomHatUpLeft() {
        return bottomHat(315);
    }

    public Trigger centerBottomHatUp() {
        return centerBottomHat(0);
    }

    public Trigger centerBottomHatUpRight() {
        return centerBottomHat(45);
    }

    public Trigger centerBottomHatRight() {
        return centerBottomHat(90);
    }

    public Trigger centerBottomHatDownRight() {
        return centerBottomHat(135);
    }

    public Trigger centerBottomHatDown() {
        return centerBottomHat(180);
    }

    public Trigger centerBottomHatDownLeft() {
        return centerBottomHat(225);
    }

    public Trigger centerBottomHatLeft() {
        return centerBottomHat(270);
    }

    public Trigger centerBottomHatUpLeft() {
        return centerBottomHat(315);
    }

    public Trigger centerTopHatUp() {
        return centerTopHat(0);
    }

    public Trigger centerTopHatUpRight() {
        return centerTopHat(45);
    }

    public Trigger centerTopHatRight() {
        return centerTopHat(90);
    }

    public Trigger centerTopHatDownRight() {
        return centerTopHat(135);
    }

    public Trigger centerTopHatDown() {
        return centerTopHat(180);
    }

    public Trigger centerTopHatDownLeft() {
        return centerTopHat(225);
    }

    public Trigger centerTopHatLeft() {
        return centerTopHat(270);
    }

    public Trigger centerTopHatUpLeft() {
        return centerTopHat(315);
    }

    public Trigger topRightHatUp() {
        return topRightHat(0);
    }

    public Trigger topRightHatDown() {
        return topRightHat(180);
    }

}
