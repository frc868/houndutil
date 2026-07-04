package com.techhounds.houndutil.houndlib.subsystems.finished;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public interface FinishedSubsystemBase {
    /**
     * <p>
     * Example method creation:
     * 
     * <blockquote><pre>
     *     &#64;Override
     *public FinishedTalonSystem[] getTalonInfo() {
     *     return new FinishedTalonSystem[] {
     *          new FinishedTalonSystem(
     *               "leftMotor",
     *               10,
     *               InvertedValue.CounterClockwise_Positive),
     *          new FinishedTalonSystem(
     *               "rightMotor",
     *               11,
     *               InvertedValue.Clockwise_Positive)
     *     };
     *}
     * </pre></blockquote><p>
     * 
     * @return An array containing a FinishedTalonSystem for each motor in the subsystem. If {@code getAreFollowers()} is true, the first motor in the array will be the master.
     */
    public FinishedTalonSystem[] getTalonInfo();

    /**
     * <p>
     * Example method creation:
     * 
     * <blockquote><pre>
     *     &#64;Override
     *public boolean getAreFollowers(){
     *     return false;
     *}
     * </pre></blockquote><p>
     * 
     * @return A boolean that describes whether the motors are in a follower configuration or are controlled independently. If they are mechanically linked, they should be followers (denoted by returning true).
     */
    public boolean getAreFollowers();

    /**
     * <p>
     * Example method creation:
     * 
     * <blockquote><pre>
     *     &#64;Override
     *public String getName(){
     *     return "shooter";
     *}
     * </pre></blockquote><p>
     * 
     * @return A String containing the name of the subsystem. This will mainly be used for the purpose of logging.
     */
    public String getName();

    /**
     * <p>
     * Example method creation:
     * 
     * <blockquote><pre>
     *     &#64;Override
     *public Current getCurrentLimit(){
     *     return Amps.of(40.0);
     *}
     * </pre></blockquote><p>
     * 
     * @return A {@code Current} object that represents the limit of electrical current allowed per motor.
     */
    public Current getCurrentLimit();

    /**
     * <p>
     * Example method creation:
     * 
     * <blockquote><pre>
     *     &#64;Override
     *public double getGearRatio(){
     *     return 7.0 / 4.0;
     *}
     * </pre></blockquote><p>
     * 
     * @return
     */
    public double getGearRatio();

    /**
     * <p>
     * Example method creation:
     * 
     * <blockquote><pre>
     *     &#64;Override
     *public NeutralModeValue getNeutral(){
     *     return NeutralModeValue.Coast;
     *}
     * </pre></blockquote><p>
     * 
     * @return
     */
    public NeutralModeValue getNeutral();

    /**
     * <p>
     * Example method creation:
     * 
     * <blockquote><pre>
     *     &#64;Override
     *public CANBus getCanBus(){
     *     return new CANBus("canivore");
     *}
     * </pre></blockquote><p>
     * 
     * @return
     */
    public CANBus getCanBus();

    /**
     * <p>
     * Example method creation:
     * 
     * <blockquote><pre>
     *     &#64;Override
     *public double[] getTuningConstants(){
     *     return new double[] { 9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
     *}
     * </pre></blockquote><p>
     * 
     * @return
     */
    public double[] getTuningConstants();

    /**
     * <p>
     * Example method creation:
     * 
     * <blockquote><pre>
     *     &#64;Override
     *public FlywheelSim getFlywheelSim() {
     *     return new FlywheelSim(
     *          LinearSystemId.createFlywheelSystem(
     *               DCMotor.getKrakenX60Foc(1),
     *               0.001,
     *               this.getGearRatio()),
     *          DCMotor.getKrakenX60Foc(1));
     *}
     * </pre></blockquote><p>
     * 
     * @return
     */
    public FlywheelSim getFlywheelSim();
}
