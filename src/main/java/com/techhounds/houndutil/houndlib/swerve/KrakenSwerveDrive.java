package com.techhounds.houndutil.houndlib.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive.DriveMode;
import com.techhounds.houndutil.houndlib.swerve.KrakenCoaxialSwerveModule.SwerveConstants;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@LoggedObject
public class KrakenSwerveDrive {
    private final KrakenCoaxialSwerveModule frontLeft;
    private final KrakenCoaxialSwerveModule frontRight;
    private final KrakenCoaxialSwerveModule backLeft;
    private final KrakenCoaxialSwerveModule backRight;
    private final Pigeon2 pigeon;
    private final DriveMode driveMode;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDrivePoseEstimator precisePoseEstimator;
    private final SwerveDriveKinematics kinematics;
    private final Orchestra orchestra = new Orchestra();

    private final ReadWriteLock stateLock = new ReentrantReadWriteLock();

    private SwerveDriveOdometry simOdometry;

    private boolean initialized = RobotBase.isSimulation();

    private final SwerveConstants constants;

    @Log(groups = "control")
    private ChassisSpeeds commandedChassisSpeeds = new ChassisSpeeds();
    @Log(groups = "control")
    private ChassisSpeeds adjustedChassisSpeeds = new ChassisSpeeds();

    @Log(groups = "control")
    private SwerveModuleState[] commandedModuleStates = new SwerveModuleState[] { new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };

    private final MutVoltage sysidDriveAppliedVoltageMeasure = Volts.mutable(0);
    private final MutDistance sysidDrivePositionMeasure = Meters.mutable(0);
    private final MutLinearVelocity sysidDriveVelocityMeasure = MetersPerSecond.mutable(0);

    private final SysIdRoutine sysIdDrive;

    private final MutVoltage sysidSteerAppliedVoltageMeasure = Volts.mutable(0);
    private final MutAngle sysidSteerPositionMeasure = Rotations.mutable(0);
    private final MutAngularVelocity sysidSteerVelocityMeasure = RotationsPerSecond.mutable(0);

    private final SysIdRoutine sysIdSteer;

    public KrakenSwerveDrive(KrakenCoaxialSwerveModule frontLeft, KrakenCoaxialSwerveModule frontRight,
            KrakenCoaxialSwerveModule backLeft, KrakenCoaxialSwerveModule backRight, Pigeon2 pigeon,
            DriveMode driveMode, SwerveDrivePoseEstimator poseEstimator, SwerveDrivePoseEstimator precisePoseEstimator,
            SwerveDriveKinematics kinematics, SwerveConstants constants, Subsystem subsystem,
            SysIdRoutine.Config sysIdConfigDrive, SysIdRoutine.Config sysIdConfigSteer) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.pigeon = pigeon;

        this.driveMode = driveMode;

        this.poseEstimator = poseEstimator;
        this.precisePoseEstimator = precisePoseEstimator;

        this.kinematics = kinematics;

        this.constants = constants;

        if (RobotBase.isSimulation()) {
            simOdometry = new SwerveDriveOdometry(kinematics, getRotation(), getModulePositions(), new Pose2d());
        }

        orchestra.addInstrument(frontLeft.getDriveMotor());
        orchestra.addInstrument(frontRight.getDriveMotor());
        orchestra.addInstrument(backLeft.getDriveMotor());
        orchestra.addInstrument(backRight.getDriveMotor());
        orchestra.addInstrument(frontLeft.getSteerMotor());
        orchestra.addInstrument(frontRight.getSteerMotor());
        orchestra.addInstrument(backLeft.getSteerMotor());
        orchestra.addInstrument(backRight.getSteerMotor());

        sysIdDrive = new SysIdRoutine(
                sysIdConfigDrive,
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            drive(new ChassisSpeeds(
                                    constants.MAX_DRIVING_VELOCITY_METERS_PER_SECOND * volts.magnitude() / 12.0,
                                    0,
                                    0));
                        },
                        log -> {
                            log.motor("frontLeft")
                                    .voltage(sysidDriveAppliedVoltageMeasure
                                            .mut_replace(frontLeft.getDriveMotorVoltage(), Volts))
                                    .linearPosition(sysidDrivePositionMeasure
                                            .mut_replace(frontLeft.getDriveMotorPosition(), Meters))
                                    .linearVelocity(sysidDriveVelocityMeasure
                                            .mut_replace(frontLeft.getDriveMotorVelocity(), MetersPerSecond));
                            log.motor("frontRight")
                                    .voltage(sysidDriveAppliedVoltageMeasure
                                            .mut_replace(frontRight.getDriveMotorVoltage(), Volts))
                                    .linearPosition(sysidDrivePositionMeasure
                                            .mut_replace(frontRight.getDriveMotorPosition(), Meters))
                                    .linearVelocity(sysidDriveVelocityMeasure
                                            .mut_replace(frontRight.getDriveMotorVelocity(), MetersPerSecond));
                            log.motor("backLeft")
                                    .voltage(sysidDriveAppliedVoltageMeasure
                                            .mut_replace(backLeft.getDriveMotorVoltage(), Volts))
                                    .linearPosition(sysidDrivePositionMeasure
                                            .mut_replace(backLeft.getDriveMotorPosition(), Meters))
                                    .linearVelocity(sysidDriveVelocityMeasure
                                            .mut_replace(backLeft.getDriveMotorVelocity(), MetersPerSecond));
                            log.motor("backRight")
                                    .voltage(sysidDriveAppliedVoltageMeasure
                                            .mut_replace(backRight.getDriveMotorVoltage(), Volts))
                                    .linearPosition(sysidDrivePositionMeasure
                                            .mut_replace(backRight.getDriveMotorPosition(), Meters))
                                    .linearVelocity(sysidDriveVelocityMeasure
                                            .mut_replace(backRight.getDriveMotorVelocity(), MetersPerSecond));
                        },
                        subsystem));

        sysIdSteer = new SysIdRoutine(
                sysIdConfigSteer,
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            drive(new ChassisSpeeds(
                                    constants.MAX_DRIVING_VELOCITY_METERS_PER_SECOND * volts.magnitude() /
                                            12.0,
                                    0,
                                    0));
                        },
                        log -> {
                            log.motor("frontLeft")
                                    .voltage(sysidSteerAppliedVoltageMeasure
                                            .mut_replace(frontLeft.getSteerMotorVoltage(), Volts))
                                    .angularPosition(sysidSteerPositionMeasure
                                            .mut_replace(frontLeft.getSteerMotorPosition(), Rotations))
                                    .angularVelocity(sysidSteerVelocityMeasure
                                            .mut_replace(frontLeft.getSteerMotorVelocity(), RotationsPerSecond));
                            log.motor("frontRight")
                                    .voltage(sysidSteerAppliedVoltageMeasure
                                            .mut_replace(frontRight.getSteerMotorVoltage(), Volts))
                                    .angularPosition(sysidSteerPositionMeasure
                                            .mut_replace(frontRight.getSteerMotorPosition(), Rotations))
                                    .angularVelocity(sysidSteerVelocityMeasure
                                            .mut_replace(frontRight.getSteerMotorVelocity(), RotationsPerSecond));
                            log.motor("backLeft")
                                    .voltage(sysidSteerAppliedVoltageMeasure
                                            .mut_replace(backLeft.getSteerMotorVoltage(), Volts))
                                    .angularPosition(sysidSteerPositionMeasure
                                            .mut_replace(backLeft.getSteerMotorPosition(), Rotations))
                                    .angularVelocity(sysidSteerVelocityMeasure
                                            .mut_replace(backLeft.getSteerMotorVelocity(), RotationsPerSecond));
                            log.motor("backRight")
                                    .voltage(sysidSteerAppliedVoltageMeasure
                                            .mut_replace(backRight.getSteerMotorVoltage(), Volts))
                                    .angularPosition(sysidSteerPositionMeasure
                                            .mut_replace(backRight.getSteerMotorPosition(), Rotations))
                                    .angularVelocity(sysidSteerVelocityMeasure
                                            .mut_replace(backRight.getSteerMotorVelocity(), RotationsPerSecond));
                        },
                        subsystem));
    }

    /**
     * Gets the currently set drive mode.
     * 
     * @return the drive mode
     */
    public DriveMode getDriveMode() {
        return driveMode;
    }

    /**
     * Gets the current (estimated) pose of the chassis, with respect to the origin.
     * 
     * @return the pose of the chassis
     */
    @Log
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the current rotation of the chassis, with respect to the origin.
     * 
     * @return the rotation of the chassis, as a Rotation2d
     */
    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
    }

    /**
     * Gets an array containing the distance travelled and the azimuth angle for
     * each swerve module. Used mainly for odometry.
     * 
     * @return the array of SwerveModulePositions
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    /**
     * Gets an array containing the current velocity and the azimuth angle for
     * each swerve module.
     * 
     * @return the array of SwerveModuleStates
     */
    @Log(groups = "control")
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    /**
     * Gets the current <i>robot-relative</i> velocity of the chassis as a whole,
     * dependent on the states of the swerve modules.
     * 
     * @return the velocity of the chassis, as a ChassisSpeeds
     */
    @Log(groups = "control")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Gets the pose estimator object for fusing latency-compensated vision
     * measurements with odometry data.
     * 
     * @return the SwerveDrivePoseEstimator object.
     */
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /**
     * Resets the pose estimator to a specific position on the field. Useful for
     * known starting locations before the autonomous period.
     * 
     * @param pose the pose to reset the chassis position to
     */
    public void resetPoseEstimator(Pose2d pose) {
        try {
            // since the pose estimator is used by another thread, we need to lock it to be
            // able to reset it
            stateLock.writeLock().lock();

            poseEstimator.resetPosition(getRotation(), getModulePositions(),
                    new Pose2d(pose.getTranslation(), getRotation()));
            precisePoseEstimator.resetPosition(getRotation(), getModulePositions(),
                    new Pose2d(pose.getTranslation(), getRotation()));
            if (RobotBase.isSimulation())
                simOdometry.resetPosition(getRotation(), getModulePositions(),
                        new Pose2d(pose.getTranslation(), getRotation()));
        } finally {
            stateLock.writeLock().unlock();
        }
    }

    /**
     * Resets the gyro such that the chassis is facing forward with respect to the
     * origin.
     */
    public void resetGyro() {
        pigeon.setYaw(0);
        initialized = true;
    }

    /**
     * Fetch the initialization status of the drivetrain.
     * 
     * @return whether the drivetrain is initialized
     */
    @Log
    public boolean getInitialized() {
        return initialized;
    }

    /**
     * Sets the motors on the swerve modules into either brake or coast mode.
     * 
     * @param neutralMode the NeutralModeValue to set the motors in each swerve
     *                    module to
     */
    public void setMotorNeutralModes(NeutralModeValue neutralMode) {
        frontLeft.setMotorNeutralMode(neutralMode);
        frontRight.setMotorNeutralMode(neutralMode);
        backLeft.setMotorNeutralMode(neutralMode);
        backRight.setMotorNeutralMode(neutralMode);
    }

    /**
     * Sets the stator current limit on each swerve module. Useful for a temporary
     * high-power mode.
     * 
     * @param currentLimit the stator current limit to set, in amps
     */
    public void setDriveCurrentLimits(int currentLimit) {
        frontLeft.setDriveCurrentLimit(currentLimit);
        frontRight.setDriveCurrentLimit(currentLimit);
        backLeft.setDriveCurrentLimit(currentLimit);
        backRight.setDriveCurrentLimit(currentLimit);

    }

    /**
     * Stops all swerve modules.
     */
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Sets the state (velocity and azimuth angle) of each swerve module, without
     * closed-loop velocity control.
     * 
     * @param state an array of SwerveModuleStates to set the modules to
     */
    public void setStates(SwerveModuleState[] states) {
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    /**
     * Sets the state (velocity and azimuth angle) of each swerve module, with
     * closed-loop velocity control.
     * 
     * @param state an array of SwerveModuleStates to set the modules to
     */
    public void setStatesClosedLoop(SwerveModuleState[] states) {
        frontLeft.setStateClosedLoop(states[0]);
        frontRight.setStateClosedLoop(states[1]);
        backLeft.setStateClosedLoop(states[2]);
        backRight.setStateClosedLoop(states[3]);
    }

    /**
     * Sets the states of the swerve modules to accomplish the given chassis speeds.
     * Uses the currently set DriveMode.
     * 
     * @param speeds the ChassisSpeeds to use to drive the swerve modules
     */
    public void drive(ChassisSpeeds speeds) {
        drive(speeds, this.driveMode);
    }

    /**
     * Sets the states of the swerve modules to accomplish the given chassis speeds.
     * 
     * @param speeds    the ChassisSpeeds to use to drive the swerve modules.
     * @param driveMode the DriveMode to use for the chassis' reference point
     */
    public void drive(ChassisSpeeds speeds, DriveMode driveMode) {
        if (Utils.shouldFlipValueToRed()
                && driveMode == DriveMode.FIELD_ORIENTED) {
            speeds.vxMetersPerSecond *= -1;
            speeds.vyMetersPerSecond *= -1;
        }

        commandedChassisSpeeds = speeds;
        ChassisSpeeds adjustedChassisSpeeds = null;
        switch (driveMode) {
            case ROBOT_RELATIVE:
                adjustedChassisSpeeds = speeds;
                break;
            case FIELD_ORIENTED:
                adjustedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond, getRotation());
                break;
        }

        // compensates for swerve skew when translating and rotating simultaneously
        adjustedChassisSpeeds = ChassisSpeeds.discretize(adjustedChassisSpeeds, 0.02);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(adjustedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                constants.MAX_DRIVING_VELOCITY_METERS_PER_SECOND);

        states[0].optimize(frontLeft.getWheelAngle());
        states[1].optimize(frontRight.getWheelAngle());
        states[2].optimize(backLeft.getWheelAngle());
        states[3].optimize(backRight.getWheelAngle());

        commandedModuleStates = states;
        setStates(states);
    }

    /**
     * Sets the states of the swerve modules to accomplish the given chassis speeds,
     * with closed-loop velocity control.
     * 
     * @param speeds    the ChassisSpeeds to use to drive the swerve modules.
     * @param driveMode the DriveMode to use for the chassis' reference point
     */
    public void driveClosedLoop(ChassisSpeeds speeds, DriveMode driveMode) {
        if (Utils.shouldFlipValueToRed()
                && driveMode == DriveMode.FIELD_ORIENTED) {
            speeds.vxMetersPerSecond *= -1;
            speeds.vyMetersPerSecond *= -1;
        }

        commandedChassisSpeeds = speeds;
        adjustedChassisSpeeds = null;
        switch (driveMode) {
            case ROBOT_RELATIVE:
                adjustedChassisSpeeds = speeds;
                break;
            case FIELD_ORIENTED:
                adjustedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond, getRotation());
                break;
        }

        adjustedChassisSpeeds = ChassisSpeeds.discretize(adjustedChassisSpeeds, 0.02);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(adjustedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                constants.MAX_DRIVING_VELOCITY_METERS_PER_SECOND);

        states[0].optimize(frontLeft.getWheelAngle());
        states[1].optimize(frontRight.getWheelAngle());
        states[2].optimize(backLeft.getWheelAngle());
        states[3].optimize(backRight.getWheelAngle());

        commandedModuleStates = states;
        setStatesClosedLoop(states);
    }

    /**
     * Exposes the orchestra to load music or play
     * 
     * @return the orchestra
     */
    public Orchestra getOrchestra() {
        return orchestra;
    }

    /**
     * Gets the SysId routine for the drive motors.
     * 
     * @return the SysIdRoutine
     */
    public SysIdRoutine getSysIdDrive() {
        return sysIdDrive;
    }

    /**
     * Gets the SysId routine for the steering motors.
     * 
     * @return the SysIdRoutine
     */
    public SysIdRoutine getSysIdSteer() {
        return sysIdSteer;
    }
}