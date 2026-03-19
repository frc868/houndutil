package com.techhounds.houndutil.houndlib.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive.DriveMode;
import com.techhounds.houndutil.houndlib.swerve.KrakenCoaxialSwerveModule.SwerveConstants;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@LoggedObject
public class KrakenSwerveDrive {
    /** The front left swerve module. */
    private final KrakenCoaxialSwerveModule frontLeft;
    /** The front right swerve module. */
    private final KrakenCoaxialSwerveModule frontRight;
    /** The back left swerve module. */
    private final KrakenCoaxialSwerveModule backLeft;
    /** The back right swerve module. */
    private final KrakenCoaxialSwerveModule backRight;

    /** The Pigeon 2.0 gyroscope, accelerometer, and magnetometer. */
    private final Pigeon2 pigeon;

    /** The currently set DriveMode of the chassis. */
    private final DriveMode driveMode;

    /** The pose estimator for the swerve drive. */
    private final SwerveDrivePoseEstimator poseEstimator;
    /** The precise pose estimator for the swerve drive. */
    private final SwerveDrivePoseEstimator precisePoseEstimator;

    /** The kinematics constants for the swerve drive. */
    private final SwerveDriveKinematics kinematics;

    /** The orchestra for playing music through the motors. */
    private final Orchestra orchestra = new Orchestra();

    /** Lock for accessing the pose estimator from multiple threads. */
    private final ReadWriteLock stateLock = new ReentrantReadWriteLock();
    /** The odometry thread for 250Hz odometry updates. */
    private final OdometryThread odometryThread;

    /** The simulation odometry, for simulation only. */
    private SwerveDriveOdometry simOdometry;

    /**
     * Whether the drivetrain has been initialized. Instantly true in sim, true only
     * after gyro reset otherwise.
     */
    private boolean initialized = RobotBase.isSimulation();

    /** The constants for the swerve drive. */
    private final SwerveConstants constants;

    /** The last commanded chassis speeds. */
    @Log(groups = "control")
    private ChassisSpeeds commandedChassisSpeeds = new ChassisSpeeds();
    /** The adjusted chassis speeds after applying the drive mode. */
    @Log(groups = "control")
    private ChassisSpeeds adjustedChassisSpeeds = new ChassisSpeeds();

    /** The last commanded module states. */
    @Log(groups = "control")
    private SwerveModuleState[] commandedModuleStates = new SwerveModuleState[] { new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };

    // SysId measurement variables
    private final MutVoltage sysidDriveAppliedVoltageMeasure = Volts.mutable(0);
    private final MutDistance sysidDrivePositionMeasure = Meters.mutable(0);
    private final MutLinearVelocity sysidDriveVelocityMeasure = MetersPerSecond.mutable(0);
    /** The sysid routine for the drive motors */
    private final SysIdRoutine sysIdDrive;

    // SysId measurement variables
    private final MutVoltage sysidSteerAppliedVoltageMeasure = Volts.mutable(0);
    private final MutAngle sysidSteerPositionMeasure = Rotations.mutable(0);
    private final MutAngularVelocity sysidSteerVelocityMeasure = RotationsPerSecond.mutable(0);
    /** The sysid routine for the steer motors */
    private final SysIdRoutine sysIdSteer;

    /** The priority for the odometry thread. */
    private final int odometryThreadPriority;

    /** The average time spent in the odometry loop. */
    @Log(groups = "odometry")
    private double averageOdometryLoopTime = 0;
    @Log(groups = "odometry")
    // DAQ = data acquisition, from CTRE's odometry thread
    private int successfulDaqs = 0;
    @Log(groups = "odometry")
    private int failedDaqs = 0;

    /**
     * Initializes a swerve drive of KrakenCoaxialSwerveModules.
     * 
     * @param frontLeft              the front left swerve module
     * @param frontRight             the front right swerve module
     * @param backLeft               the back left swerve module
     * @param backRight              the back right swerve module
     * @param pigeon                 the Pigeon 2.0
     *                               gyroscope/accelerometer/magnetometer
     * @param driveMode              the DriveMode
     * @param kinematics             the SwerveDriveKinematics
     * @param constants              the SwerveConstants
     * @param subsystem              the Subsystem this swerve drive belongs to
     * @param sysIdConfigDrive       the SysId config for the drive motors
     * @param sysIdConfigSteer       the SysId config for the steer motors
     * @param odometryThreadPriority the priority for the odometry thread, with 1
     *                               being the minimum and sufficient for tighter
     *                               odometry loops, and numbers above being used in
     *                               case the odometry period is far away from the
     *                               desired frequency
     */
    public KrakenSwerveDrive(KrakenCoaxialSwerveModule frontLeft, KrakenCoaxialSwerveModule frontRight,
            KrakenCoaxialSwerveModule backLeft, KrakenCoaxialSwerveModule backRight, Pigeon2 pigeon,
            DriveMode driveMode, SwerveDriveKinematics kinematics, SwerveConstants constants, Subsystem subsystem,
            SysIdRoutine.Config sysIdConfigDrive, SysIdRoutine.Config sysIdConfigSteer, int odometryThreadPriority) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.pigeon = pigeon;

        this.driveMode = driveMode;

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getRotation(),
                getModulePositions(),
                new Pose2d(0, 0, Rotation2d.kZero));
        precisePoseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getRotation(),
                getModulePositions(),
                new Pose2d(0, 0, Rotation2d.kZero));

        this.kinematics = kinematics;

        this.constants = constants;

        this.odometryThreadPriority = odometryThreadPriority;

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
                                    0,
                                    0, // TODO this should really be the max theta velocity
                                    constants.MAX_DRIVING_VELOCITY.in(MetersPerSecond) * volts.magnitude() / 12.0));
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
                            frontLeft.getSteerMotor().setControl(new VoltageOut(volts));
                            frontRight.getSteerMotor().setControl(new VoltageOut(volts));
                            backLeft.getSteerMotor().setControl(new VoltageOut(volts));
                            backRight.getSteerMotor().setControl(new VoltageOut(volts));
                        },
                        log -> {
                            log.motor("frontLeft")
                                    .voltage(sysidSteerAppliedVoltageMeasure
                                            .mut_replace(frontLeft.getSteerMotor().getMotorVoltage().getValueAsDouble(), Volts))
                                    .angularPosition(sysidSteerPositionMeasure
                                            .mut_replace(frontLeft.getSteerMotor().getPosition().getValueAsDouble(), Rotations))
                                    .angularVelocity(sysidSteerVelocityMeasure
                                            .mut_replace(frontLeft.getSteerMotorVelocity(), RotationsPerSecond));
                            log.motor("frontRight")
                                    .voltage(sysidSteerAppliedVoltageMeasure
                                            .mut_replace(frontRight.getSteerMotor().getMotorVoltage().getValueAsDouble(), Volts))
                                    .angularPosition(sysidSteerPositionMeasure
                                            .mut_replace(frontRight.getSteerMotor().getPosition().getValueAsDouble(), Rotations))
                                    .angularVelocity(sysidSteerVelocityMeasure
                                            .mut_replace(frontRight.getSteerMotor().getVelocity().getValueAsDouble(), RotationsPerSecond));
                            log.motor("backLeft")
                                    .voltage(sysidSteerAppliedVoltageMeasure
                                            .mut_replace(backLeft.getSteerMotor().getMotorVoltage().getValueAsDouble(), Volts))
                                    .angularPosition(sysidSteerPositionMeasure
                                            .mut_replace(backLeft.getSteerMotor().getPosition().getValueAsDouble(), Rotations))
                                    .angularVelocity(sysidSteerVelocityMeasure
                                            .mut_replace(backLeft.getSteerMotorVelocity(), RotationsPerSecond));
                            log.motor("backRight")
                                    .voltage(sysidSteerAppliedVoltageMeasure
                                            .mut_replace(backRight.getSteerMotor().getMotorVoltage().getValueAsDouble(), Volts))
                                    .angularPosition(sysidSteerPositionMeasure
                                            .mut_replace(backRight.getSteerMotor().getPosition().getValueAsDouble(), Rotations))
                                    .angularVelocity(sysidSteerVelocityMeasure
                                            .mut_replace(backRight.getSteerMotorVelocity(), RotationsPerSecond));
                        },
                        subsystem));

        odometryThread = new OdometryThread();
        odometryThread.start();

        AutoManager.getInstance().setResetOdometryConsumer(this::resetPoseEstimator);
    }

    /**
     * Thread enabling 250Hz odometry. Optimized from CTRE's internal swerve code.
     * 250Hz odometry reduces discretization error in the odometry loop, and
     * significantly improves odometry during high speed maneuvers.
     */
    public class OdometryThread {
        // Testing shows 1 (minimum realtime) is sufficient for tighter odometry loops.
        // If the odometry period is far away from the desired frequency, increasing
        // the priority may help

        private final Thread m_thread;
        private volatile boolean m_running = false;

        private final BaseStatusSignal[] allSignals;

        private final MedianFilter peakRemover = new MedianFilter(3);
        private final LinearFilter lowPass = LinearFilter.movingAverage(50);
        private double lastTime = 0;
        private double currentTime = 0;

        private KrakenCoaxialSwerveModule[] modules = new KrakenCoaxialSwerveModule[] {
                frontLeft, frontRight, backLeft, backRight };
        private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

        private int lastThreadPriority = odometryThreadPriority;
        private volatile int threadPriorityToSet = odometryThreadPriority;
        private final int UPDATE_FREQUENCY = 250;

        public OdometryThread() {
            m_thread = new Thread(this::run);
            /*
             * Mark this thread as a "daemon" (background) thread
             * so it doesn't hold up program shutdown
             */
            m_thread.setDaemon(true);

            /* 4 signals for each module + 2 for Pigeon2 */

            allSignals = new BaseStatusSignal[(4 * 4) + 2];
            for (int i = 0; i < 4; ++i) {
                BaseStatusSignal[] signals = modules[i].getSignals();
                allSignals[(i * 4) + 0] = signals[0];
                allSignals[(i * 4) + 1] = signals[1];
                allSignals[(i * 4) + 2] = signals[2];
                allSignals[(i * 4) + 3] = signals[3];
            }
            allSignals[allSignals.length - 2] = pigeon.getYaw();
            allSignals[allSignals.length - 1] = pigeon.getAngularVelocityZWorld();
        }

        /**
         * Starts the odometry thread.
         */
        public void start() {
            m_running = true;
            m_thread.start();
        }

        /**
         * Stops the odometry thread.
         */
        public void stop() {
            stop(0);
        }

        /**
         * Stops the odometry thread with a timeout.
         *
         * @param millis The time to wait in milliseconds
         */
        public void stop(long millis) {
            m_running = false;
            try {
                m_thread.join(millis);
            } catch (final InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }

        public void run() {
            /* Make sure all signals update at the correct update frequency */
            BaseStatusSignal.setUpdateFrequencyForAll(UPDATE_FREQUENCY, allSignals);
            Threads.setCurrentThreadPriority(true, odometryThreadPriority);

            /* Run as fast as possible, our signals will control the timing */
            while (m_running) {
                /* Synchronously wait for all signals in drivetrain */
                /* Wait up to twice the period of the update frequency */
                StatusCode status;
                status = BaseStatusSignal.waitForAll(2.0 / UPDATE_FREQUENCY, allSignals);

                try {
                    stateLock.writeLock().lock();

                    lastTime = currentTime;
                    currentTime = RobotController.getFPGATime();
                    /*
                     * We don't care about the peaks, as they correspond to GC events, and we want
                     * the period generally low passed
                     */
                    averageOdometryLoopTime = lowPass.calculate(peakRemover.calculate((currentTime - lastTime) / 1000));

                    /* Get status of first element */
                    if (status.isOK()) {
                        successfulDaqs++;
                    } else {
                        failedDaqs++;
                    }

                    /* Now update odometry */
                    /* Keep track of the change in azimuth rotations */
                    for (int i = 0; i < 4; ++i) {
                        modulePositions[i] = modules[i].getPosition();
                    }
                    double yawDegrees = BaseStatusSignal.getLatencyCompensatedValue(
                            pigeon.getYaw(), pigeon.getAngularVelocityZWorld()).magnitude();

                    /* Keep track of previous and current pose to account for the carpet vector */
                    poseEstimator.update(Rotation2d.fromDegrees(yawDegrees), modulePositions);
                    precisePoseEstimator.update(Rotation2d.fromDegrees(yawDegrees),
                            modulePositions);
                    if (RobotBase.isSimulation()) {
                        simOdometry.update(Rotation2d.fromDegrees(yawDegrees), modulePositions);
                    }
                } finally {
                    stateLock.writeLock().unlock();
                }

                /**
                 * This is inherently synchronous, since lastThreadPriority
                 * is only written here and threadPriorityToSet is only read here
                 */
                if (threadPriorityToSet != lastThreadPriority) {
                    Threads.setCurrentThreadPriority(true, threadPriorityToSet);
                    lastThreadPriority = threadPriorityToSet;
                }
            }
        }

        /**
         * Sets the DAQ thread priority to a real time priority under the specified
         * priority level
         *
         * @param priority Priority level to set the DAQ thread to.
         *                 This is a value between 0 and 99, with 99 indicating higher
         *                 priority and 0 indicating lower priority.
         */
        public void setThreadPriority(int priority) {
            threadPriorityToSet = priority;
        }
    }

    /**
     * Gets the average time taken for each odometry loop, in seconds.
     * 
     * @return the average odometry loop time
     */
    public double getOdometryLoopTime() {
        return averageOdometryLoopTime;
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
     * Gets the current (estimated) precise pose of the chassis, with respect to the
     * origin.
     * 
     * @return the precise pose of the chassis
     */
    @Log
    public Pose2d getPrecisePose() {
        return precisePoseEstimator.getEstimatedPosition();
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
    @Log(groups = "control")
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
                constants.MAX_DRIVING_VELOCITY.in(MetersPerSecond));

        states[0].optimize(frontLeft.getWheelAngle());
        states[1].optimize(frontRight.getWheelAngle());
        states[2].optimize(backLeft.getWheelAngle());
        states[3].optimize(backRight.getWheelAngle());

        commandedModuleStates = states;
        setStates(states);
    }

    public void driveClosedLoop(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        // TODO implement
        driveClosedLoop(speeds, DriveMode.ROBOT_RELATIVE);
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
                constants.MAX_DRIVING_VELOCITY.in(MetersPerSecond));

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

    /**
     * Use the current module states to get the field-relative speeds of the robot.
     * 
     * @return the field-relative chassis speeds
     */
    @Log(groups = "control")
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(getModuleStates()), getRotation());
    }

    /**
     * Adds a vision measurement to the pose estimator. Used by a vision
     * subsystem.
     * 
     * @param visionRobotPoseMeters    the estimated robot pose from vision
     * @param timestampSeconds         the timestamp of the vision measurement
     * @param visionMeasurementStdDevs the standard deviations of the measurement
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            // since the pose estimator is used by another thread, we need to lock it to be
            // able to add a vision measurement
            stateLock.writeLock().lock();
            poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
        } finally {
            stateLock.writeLock().unlock();
        }

    }

    /**
     * Adds a precise vision measurement to the pose estimator. Used by a vision
     * subsystem.
     * 
     * @param visionRobotPoseMeters    the estimated robot pose from vision
     * @param timestampSeconds         the timestamp of the vision measurement
     * @param visionMeasurementStdDevs the standard deviations of the measurement
     */
    public void addPreciseVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            // since the pose estimator is used by another thread, we need to lock it to be
            // able to add a vision measurement
            stateLock.writeLock().lock();
            precisePoseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds,
                    visionMeasurementStdDevs);
        } finally {
            stateLock.writeLock().unlock();
        }
    }

    /**
     * Draws the robot on a Field2d. This will include the angles of the swerve
     * modules on the outsides of the robot box in Glass.
     * 
     * @param field the field to draw the robot on (usually
     *              {@code AutoManager.getInstance().getField()})
     */
    public void drawRobotOnField(Field2d field) {
        field.setRobotPose(getPose());
        if (RobotBase.isSimulation())
            field.getObject("simPose").setPose(simOdometry.getPoseMeters());
        field.getObject("precisePose").setPose(precisePoseEstimator.getEstimatedPosition());
    }

    /**
     * Get the yaw of the Pigeon's gyro.
     * 
     * @return the yaw as an Angle
     */
    public Angle getRawYaw() {
        return pigeon.getYaw().getValue();
    }

    /**
     * Gets the simulated pose of the robot. Only works in simulation.
     * 
     * @return the simulated pose
     */
    public Pose2d getSimPose() {
        if (simOdometry != null)
            return simOdometry.getPoseMeters();
        else
            return new Pose2d();
    }
}