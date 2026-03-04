package com.techhounds.houndutil.houndlib.oi;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A version of {@link VirpilAlphaPrime} with {@link Trigger} factories for
 * command-based.
 * 
 * @see VirpilAlphaPrime
 */
public class CommandVirpilAlphaPrime extends CommandGenericHID {
    /** The internal joystick object (access using {@code getHID()}) */
    private final VirpilAlphaPrime joystick;

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is
     *             plugged into.
     */
    public CommandVirpilAlphaPrime(int port) {
        super(port);

        joystick = new VirpilAlphaPrime(port);
    }

    /**
     * Get the underlying VirpilAlphaPrime object.
     *
     * @return the wrapped VirpilAlphaPrime object
     */
    @Override
    public VirpilAlphaPrime getHID() {
        return joystick;
    }

    /**
     * Get the value of the X axis.
     *
     * @return The X axis value.
     */
    public double getX() {
        return joystick.getX();
    }

    /**
     * Get the value of the Y axis.
     *
     * @return The Y axis value.
     */
    public double getY() {
        return joystick.getY();
    }

    /**
     * Get the value of the twist (Z) axis.
     *
     * @return The twist axis value.
     */
    public double getTwist() {
        return joystick.getTwist();
    }

    /**
     * Get the value of the small joystick X axis.
     *
     * @return The small joystick X axis value.
     */
    public double getStickX() {
        return joystick.getStickX();
    }

    /**
     * Get the value of the small joystick Y axis.
     *
     * @return The small joystick Y axis value.
     */
    public double getStickY() {
        return joystick.getStickY();
    }

    /**
     * Get the value of the lever axis.
     *
     * @return The lever axis value.
     */
    public double getLever() {
        return joystick.getLever();
    }

    /**
     * Constructs an event instance around the left black button's digital signal.
     *
     * @return an event instance representing the left black thumb button's digital
     *         signal
     *         attached to the {@link CommandScheduler#getDefaultButtonLoop()
     *         default scheduler button loop}.
     * @see #blackLeftThumbButton(EventLoop)
     */
    public Trigger blackLeftThumbButton() {
        return blackLeftThumbButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the left black button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the black thumb button's digital
     *         signal
     *         attached to the given loop.
     */
    public Trigger blackLeftThumbButton(EventLoop loop) {
        return joystick.blackLeftThumbButton(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the right black thumb button's digital
     * signal.
     *
     * @return an event instance representing the right black thumb button's digital
     *         signal
     *         attached to the {@link CommandScheduler#getDefaultButtonLoop()
     *         default scheduler button loop}.
     * @see #blackRightThumbButton(EventLoop)
     */
    public Trigger blackRightThumbButton() {
        return blackRightThumbButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the right black thumb button's digital
     * signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right black thumb button's digital
     *         signal
     *         attached to the given loop.
     */
    public Trigger blackRightThumbButton(EventLoop loop) {
        return joystick.blackRightThumbButton(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the pinkie button's digital signal.
     *
     * @return an event instance representing the pinkie button's digital signal
     *         attached to the {@link CommandScheduler#getDefaultButtonLoop()
     *         default scheduler button loop}.
     * @see #redButton(EventLoop)
     */
    public Trigger pinkieButton() {
        return pinkieButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the pinkie button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the pinkie button's digital signal
     *         attached to the given loop.
     */
    public Trigger pinkieButton(EventLoop loop) {
        return joystick.pinkieButton(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the trigger soft press's digital signal.
     *
     * @return an event instance representing the trigger soft press's digital
     *         signal attached to the {@link CommandScheduler#getDefaultButtonLoop()
     *         default scheduler button loop}.
     * @see #redButton(EventLoop)
     */
    public Trigger triggerSoftPress() {
        return triggerSoftPress(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the trigger soft press's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the trigger soft press's digital
     *         signal attached to the given loop.
     */
    public Trigger triggerSoftPress(EventLoop loop) {
        return joystick.triggerSoftPress(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the trigger hard press's digital signal.
     *
     * @return an event instance representing the trigger hard press's digital
     *         signal attached to the {@link CommandScheduler#getDefaultButtonLoop()
     *         default scheduler button loop}.
     * @see #redButton(EventLoop)
     */
    public Trigger triggerHardPress() {
        return triggerHardPress(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the trigger hard press's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the trigger hard press's digital
     *         signal attached to the given loop.
     */
    public Trigger triggerHardPress(EventLoop loop) {
        return joystick.triggerHardPress(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the flip trigger out's digital signal.
     *
     * @return an event instance representing the flip trigger out's digital
     *         signal attached to the {@link CommandScheduler#getDefaultButtonLoop()
     *         default scheduler button loop}.
     * @see #redButton(EventLoop)
     */
    public Trigger flipTriggerOut() {
        return flipTriggerOut(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the flip trigger out's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the flip trigger out's digital
     *         signal attached to the given loop.
     */
    public Trigger flipTriggerOut(EventLoop loop) {
        return joystick.flipTriggerOut(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the flip trigger in's digital signal.
     *
     * @return an event instance representing the flip trigger in's digital
     *         signal attached to the {@link CommandScheduler#getDefaultButtonLoop()
     *         default scheduler button loop}.
     * @see #redButton(EventLoop)
     */
    public Trigger flipTriggerIn() {
        return flipTriggerIn(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the flip trigger in's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the flip trigger in's digital
     *         signal attached to the given loop.
     */
    public Trigger flipTriggerIn(EventLoop loop) {
        return joystick.flipTriggerIn(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the stick button's digital signal.
     *
     * @return an event instance representing the stick button's digital
     *         signal attached to the {@link CommandScheduler#getDefaultButtonLoop()
     *         default scheduler button loop}.
     * @see #redButton(EventLoop)
     */
    public Trigger stickButton() {
        return stickButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the stick button's digital
     *         signal attached to the given loop.
     */
    public Trigger stickButton(EventLoop loop) {
        return joystick.stickButton(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the dial soft press's digital signal.
     *
     * @return an event instance representing the dial soft press's digital
     *         signal attached to the {@link CommandScheduler#getDefaultButtonLoop()
     *         default scheduler button loop}.
     * @see #redButton(EventLoop)
     */
    public Trigger dialSoftPress() {
        return dialSoftPress(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the dial soft press's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the dial soft press's digital
     *         signal attached to the given loop.
     */
    public Trigger dialSoftPress(EventLoop loop) {
        return joystick.dialSoftPress(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the dial hard press's digital signal.
     *
     * @return an event instance representing the dial hard press's digital
     *         signal attached to the {@link CommandScheduler#getDefaultButtonLoop()
     *         default scheduler button loop}.
     * @see #redButton(EventLoop)
     */
    public Trigger dialHardPress() {
        return dialHardPress(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the dial hard press's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the dial hard press's digital
     *         signal attached to the given loop.
     */
    public Trigger dialHardPress(EventLoop loop) {
        return joystick.dialHardPress(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the bottom HAT button's digital signal.
     *
     * @return an event instance representing the bottom HAT button's digital
     *         signal attached to the {@link CommandScheduler#getDefaultButtonLoop()
     *         default scheduler button loop}.
     * @see #redButton(EventLoop)
     */
    public Trigger bottomHatButton() {
        return bottomHatButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the bottom HAT button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the bottom HAT button's digital
     *         signal attached to the given loop.
     */
    public Trigger bottomHatButton(EventLoop loop) {
        return joystick.bottomHatButton(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the center bottom HAT button's digital
     * signal.
     *
     * @return an event instance representing the center bottom HAT button's digital
     *         signal attached to the {@link CommandScheduler#getDefaultButtonLoop()
     *         default scheduler button loop}.
     * @see #redButton(EventLoop)
     */
    public Trigger centerBottomHatButton() {
        return centerBottomHatButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the center bottom HAT button's digital
     * signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the center bottom HAT button's digital
     *         signal attached to the given loop.
     */
    public Trigger centerBottomHatButton(EventLoop loop) {
        return joystick.centerBottomHatButton(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the center top HAT button's digital
     * signal.
     *
     * @return an event instance representing the center top HAT button's digital
     *         signal attached to the {@link CommandScheduler#getDefaultButtonLoop()
     *         default scheduler button loop}.
     * @see #redButton(EventLoop)
     */
    public Trigger centerTopHatButton() {
        return centerTopHatButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the center top HAT button's digital
     * signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the center top HAT button's digital
     *         signal attached to the given loop.
     */
    public Trigger centerTopHatButton(EventLoop loop) {
        return joystick.centerTopHatButton(loop).castTo(Trigger::new);
    }

    /**
     * Constructs an event instance around the top right HAT button's digital
     * signal.
     *
     * @return an event instance representing the top right HAT button's digital
     *         signal attached to the {@link CommandScheduler#getDefaultButtonLoop()
     *         default scheduler button loop}.
     * @see #redButton(EventLoop)
     */
    public Trigger topRightHatButton() {
        return topRightHatButton(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the top right HAT button's digital
     * signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the top right HAT button's digital
     *         signal attached to the given loop.
     */
    public Trigger topRightHatButton(EventLoop loop) {
        return joystick.topRightHatButton(loop).castTo(Trigger::new);
    }

    // /**
    //  * Constructs an event instance around the top right HAT button's digital
    //  * signal.
    //  *
    //  * @return an event instance representing the bottom HAT up's digital
    //  *         signal attached to the {@link CommandScheduler#getDefaultButtonLoop()
    //  *         default scheduler button loop}.
    //  * @see #redButton(EventLoop)
    //  */
    // public Trigger bottomHatUp() {
    //     return bottomHatUp(CommandScheduler.getInstance().getDefaultButtonLoop());
    // }

    // /**
    //  * Constructs a Trigger instance based around the 0 degree angle (up) of the
    //  * bottom HAT on the Controller, attached to 
    //  * @param loop the event loop instance to attach the event to.
    //  * @return a Trigger instance based around the 0 degree angle of the bottom HAT
    //  *         on the
    //  *         Controller.
    //  */
    // public Trigger bottomHatUp(EventLoop loop) {
    //     return joystick.bottomHatUp(loop).castTo(Trigger::new);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 45 degree angle (up right) of
    //  * the
    //  * bottom HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 45 degree angle of the bottom HAT
    //  *         on the Controller.
    //  */
    // // public Trigger bottomHatUpRight() {
    // //     return bottomHat(45);
    // // } // TODO is this possible

    // /**
    //  * Constructs a Trigger instance based around the 90 degree angle (right) of the
    //  * bottom HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 90 degree angle of the bottom HAT
    //  *         on the Controller.
    //  */
    // public Trigger bottomHatRight() {
    //     return bottomHat(90);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 135 degree angle (down right)
    //  * of the
    //  * bottom HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 135 degree angle of the bottom
    //  *         HAT on the Controller.
    //  */
    // public Trigger bottomHatDownRight() {
    //     return bottomHat(135);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 180 degree angle (down) of the
    //  * bottom HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 180 degree angle of the bottom
    //  *         HAT on the Controller.
    //  */
    // public Trigger bottomHatDown() {
    //     return bottomHat(180);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 225 degree angle (down left)
    //  * of the
    //  * bottom HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 225 degree angle of the bottom
    //  *         HAT on the Controller.
    //  */
    // public Trigger bottomHatDownLeft() {
    //     return bottomHat(225);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 270 degree angle (left)
    //  * of the
    //  * bottom HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 270 degree angle of the bottom
    //  *         HAT on the Controller.
    //  */
    // public Trigger bottomHatLeft() {
    //     return bottomHat(270);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 315 degree angle (up left)
    //  * of the
    //  * bottom HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 315 degree angle of the bottom
    //  *         HAT on the Controller.
    //  */
    // public Trigger bottomHatUpLeft() {
    //     return bottomHat(315);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 0 degree angle (up)
    //  * of the
    //  * center bottom HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 0 degree angle of the center
    //  *         bottom
    //  *         HAT on the Controller.
    //  */
    // public Trigger centerBottomHatUp() {
    //     return centerBottomHat(0);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 45 degree angle (up right)
    //  * of the
    //  * center bottom HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 45 degree angle of the center
    //  *         bottom
    //  *         HAT on the Controller.
    //  */
    // public Trigger centerBottomHatUpRight() {
    //     return centerBottomHat(45);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 90 degree angle (right)
    //  * of the
    //  * center bottom HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 90 degree angle of the center
    //  *         bottom
    //  *         HAT on the Controller.
    //  */
    // public Trigger centerBottomHatRight() {
    //     return centerBottomHat(90);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 135 degree angle (down right)
    //  * of the
    //  * center bottom HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 135 degree angle of the center
    //  *         bottom
    //  *         HAT on the Controller.
    //  */
    // public Trigger centerBottomHatDownRight() {
    //     return centerBottomHat(135);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 180 degree angle (down)
    //  * of the
    //  * center bottom HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 180 degree angle of the center
    //  *         bottom
    //  *         HAT on the Controller.
    //  */
    // public Trigger centerBottomHatDown() {
    //     return centerBottomHat(180);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 225 degree angle (down left)
    //  * of the
    //  * center bottom HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 225 degree angle of the center
    //  *         bottom
    //  *         HAT on the Controller.
    //  */
    // public Trigger centerBottomHatDownLeft() {
    //     return centerBottomHat(225);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 270 degree angle (left)
    //  * of the
    //  * center bottom HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 270 degree angle of the center
    //  *         bottom
    //  *         HAT on the Controller.
    //  */
    // public Trigger centerBottomHatLeft() {
    //     return centerBottomHat(270);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 315 degree angle (up left)
    //  * of the
    //  * center bottom HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 315 degree angle of the center
    //  *         bottom
    //  *         HAT on the Controller.
    //  */
    // public Trigger centerBottomHatUpLeft() {
    //     return centerBottomHat(315);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 0 degree angle (up)
    //  * of the
    //  * center top HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 0 degree angle of the center top
    //  *         HAT on the Controller.
    //  */
    // public Trigger centerTopHatUp() {
    //     return centerTopHat(0);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 45 degree angle (up right)
    //  * of the
    //  * center top HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 45 degree angle of the center top
    //  *         HAT on the Controller.
    //  */
    // public Trigger centerTopHatUpRight() {
    //     return centerTopHat(45);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 90 degree angle (right)
    //  * of the
    //  * center top HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 90 degree angle of the center top
    //  *         HAT on the Controller.
    //  */
    // public Trigger centerTopHatRight() {
    //     return centerTopHat(90);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 135 degree angle (down right)
    //  * of the
    //  * center top HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 135 degree angle of the center
    //  *         top
    //  *         HAT on the Controller.
    //  */
    // public Trigger centerTopHatDownRight() {
    //     return centerTopHat(135);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 180 degree angle (down)
    //  * of the
    //  * center top HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 180 degree angle of the center
    //  *         top
    //  *         HAT on the Controller.
    //  */
    // public Trigger centerTopHatDown() {
    //     return centerTopHat(180);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 225 degree angle (down left)
    //  * of the
    //  * center top HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 225 degree angle of the center
    //  *         top
    //  *         HAT on the Controller.
    //  */
    // public Trigger centerTopHatDownLeft() {
    //     return centerTopHat(225);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 270 degree angle (left)
    //  * of the
    //  * center top HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 270 degree angle of the center
    //  *         top
    //  *         HAT on the Controller.
    //  */
    // public Trigger centerTopHatLeft() {
    //     return centerTopHat(270);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 315 degree angle (up left)
    //  * of the
    //  * center top HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 315 degree angle of the center
    //  *         top
    //  *         HAT on the Controller.
    //  */
    // public Trigger centerTopHatUpLeft() {
    //     return centerTopHat(315);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 0 degree angle (up)
    //  * of the
    //  * top right HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 0 degree angle of the top right
    //  *         HAT on the Controller.
    //  */
    // public Trigger topRightHatUp() {
    //     return topRightHat(0);
    // }

    // /**
    //  * Constructs a Trigger instance based around the 180 degree angle (down)
    //  * of the
    //  * top right HAT on the Controller, attached to
    //  * {@link CommandScheduler#getDefaultButtonLoop()} the default command scheduler
    //  * button loop}.
    //  *
    //  * @return a Trigger instance based around the 180 degree angle of the top right
    //  *         HAT on the Controller.
    //  */
    // public Trigger topRightHatDown() {
    //     return topRightHat(180);
    // }

    public Trigger centerTopHatUp() {
        return joystick.centerTopHatUp(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    public Trigger centerTopHatLeft() {
        return joystick.centerTopHatLeft(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    public Trigger centerTopHatRight() {
        return joystick.centerTopHatRight(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    public Trigger centerTopHatDown() {
        return joystick.centerTopHatDown(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    public Trigger centerBottomHatUp() {
        return joystick.centerBottomHatUp(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    public Trigger centerBottomHatLeft() {
        return joystick.centerBottomHatLeft(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    public Trigger centerBottomHatRight() {
        return joystick.centerBottomHatRight(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    public Trigger centerBottomHatDown() {
        return joystick.centerBottomHatDown(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    public Trigger bottomHatUp() {
        return joystick.bottomHatUp(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }
    public Trigger bottomHatLeft() {
        return joystick.bottomHatLeft(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    public Trigger bottomHatRight() {
        return joystick.bottomHatRight(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    public Trigger bottomHatDown() {
        return joystick.bottomHatDown(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    public Trigger topRightHatUp() {
        return joystick.topRightHatUp(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    public Trigger topRightHatDown() {
        return joystick.topRightHatDown(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    public Trigger flipTriggerNeutral() {
        return joystick.flipTriggerNeutral(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    public Trigger dialUp() {
        return joystick.dialUp(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    public Trigger dialDown() {
        return joystick.dialDown(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    public Trigger actuateLever() {
        return joystick.actuateLever(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    // TODO all of these new methods need javadocs, it's possible I missed some, they need to have two methods (one with param loop and another with default), it's not formatted, and they are in a random order
}
