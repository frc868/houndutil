package com.techhounds.houndutil.houndlib.subsystems.finished;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.ChassisAccelerations;
import com.techhounds.houndutil.houndlib.TuningConstants;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive.DriveMode;
import com.techhounds.houndutil.houndlib.swerve.KrakenCoaxialSwerveModule;
import com.techhounds.houndutil.houndlib.swerve.KrakenCoaxialSwerveModule.SwerveConstants;
import com.techhounds.houndutil.houndlib.swerve.KrakenSwerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public abstract class FinishedDrivetrain extends SubsystemBase {

    public static class ModuleConstants {
        public final int DRIVE_MOTOR_ID;
        public final int STEER_MOTOR_ID;
        public final int STEER_ENCODER_ID;
        public final Angle ENCODER_OFFSET;

        public ModuleConstants(int driveMotorId, int steerMotorId, int steerEncoderId, Angle encoderOffset) {
            this.DRIVE_MOTOR_ID = driveMotorId;
            this.STEER_MOTOR_ID = steerMotorId;
            this.STEER_ENCODER_ID = steerEncoderId;
            this.ENCODER_OFFSET = encoderOffset;
        }
    }

    public static class JoystickConstants {
        public final double INPUT_RATE_LIMIT;
        public final double INPUT_DEADBAND;
        public final double CURVE_EXP;
        public final double ROT_CURVE_EXP;

        public JoystickConstants(double inputRateLimit, double inputDeadband, double curveExp, double rotCurveExp) {
            this.INPUT_RATE_LIMIT = inputRateLimit;
            this.INPUT_DEADBAND = inputDeadband;
            this.CURVE_EXP = curveExp;
            this.ROT_CURVE_EXP = rotCurveExp;
        }
    }

    public final ModuleConstants[] MODULE_CONSTANTS;
    public final JoystickConstants JOYSTICK_CONSTANTS;
    public final SwerveConstants SWERVE_CONSTANTS;

    public final int PIGEON_ID = 0;
    public final CANBus CAN_BUS;

    public final boolean DRIVE_MOTORS_INVERTED;
    public final boolean STEER_MOTORS_INVERTED;
    public final boolean STEER_ENCODERS_INVERTED;

    public final Distance WHEEL_RADIUS;
    public final double WHEEL_COF;

    /** The priority to start the 250Hz odometry thread */
    public final int ODOMETRY_THREAD_PRIORITY;

    public final TuningConstants XY_PID;
    public final TuningConstants THETA_PID;
    public final AngularVelocity THETA_MAX_VELOCITY;
    public final AngularAcceleration THETA_MAX_ACCELERATION;

    public final Distance DRIVE_POSITION_TOLERANCE;
    public final LinearVelocity DRIVE_VELOCITY_TOLERANCE;
    public final Angle ROTATION_POSITION_TOLERANCE;
    public final AngularVelocity ROTATION_VELOCITY_TOLERANCE;

    /** The P gain to use when translating along a trajectory */
    public final double PATH_FOLLOWING_TRANSLATION_kP = RobotBase.isReal() ? 3.0 : 3.0;
    /** The P gain to use when rotating along a trajectory */
    public final double PATH_FOLLOWING_ROTATION_kP = RobotBase.isReal() ? 3.0 : 2.5;

    // TODO make javadocs for these 2
    public final double XY_FF_MIN_RANGE;
    public final double XY_FF_MAX_RANGE; //TODO probably make these distances

    /** Distance between left and right wheels */
    public final Distance TRACK_WIDTH;
    /** Distance between front and back wheels */
    public final Distance WHEEL_BASE;
    /** The distance between the bot center and swerve modules */
    public final Distance DRIVE_BASE_RADIUS;

    public final Translation2d[] SWERVE_MODULE_LOCATIONS;
    public final SwerveDriveKinematics KINEMATICS;

    public final Mass ROBOT_MASS;
    public final MomentOfInertia ROBOT_MOI;

    /** The time from an output being commanded to it being executed */
    public final Time PHASE_DELAY;

    /** The SysId routine run by all drive motors */
    public final SysIdRoutine.Config DRIVE_SYSID_CONFIG = new SysIdRoutine.Config(Volts.of(1).per(Second),
            Volts.of(7), Seconds.of(10), null);
    /** The SysId routine run by all steer motors */
    public final SysIdRoutine.Config STEER_SYSID_CONFIG = new SysIdRoutine.Config(Volts.of(1).per(Second),
            Volts.of(7), Seconds.of(10), null);

    /**
     * The PathPlanner configuration of the robot including information on how it
     * interacts with the field
     */
    public final RobotConfig ROBOT_CONFIG;
    public DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    /** Used for driving to a specific pose */
    private double driveToPoseDistance = 0.0;
    /** Used for driving to a specific pose */
    private double driveToPoseScalar = 0.0;

    private boolean isControlledRotationEnabled;

    private final ProfiledPIDController rotationController;
    private final ProfiledPIDController driveController;

    private final KrakenCoaxialSwerveModule[] modules;
    public final KrakenSwerveDrive swerve;

    private final Pigeon2 pigeon;
    private SwerveModulePosition[] lastModulePositions;
    private ChassisSpeeds lastFieldRelativeChassisSpeeds; // TODO probably set these
    private boolean initialized = RobotBase.isSimulation();

    /** Initialize the drivetrain */
    public FinishedDrivetrain(CANBus bus, SwerveConstants swerveConstants,
            JoystickConstants joystickConstants, ModuleConstants[] moduleConstants,
            Pair<Distance, LinearVelocity> driveTolerances, Pair<Angle, AngularVelocity> rotationTolerances,
            int odometryThreadPriority, Time phaseDelay, Mass mass, MomentOfInertia moi, boolean driveInvert,
            boolean steerInvert, boolean encoderInvert, Pair<Distance, Distance> dimensions,
            Pair<AngularVelocity, AngularAcceleration> thetaLimits, TuningConstants thetaPid, double wheelCof,
            Distance wheelRadius, Pair<Double, Double> feedforwardRange, TuningConstants robotPID, Vector<N3> pigeonPitchYawRoll) {
        this.CAN_BUS = bus;
        this.DRIVE_MOTORS_INVERTED = driveInvert;
        this.STEER_MOTORS_INVERTED = steerInvert;
        this.STEER_ENCODERS_INVERTED = encoderInvert;
        this.SWERVE_CONSTANTS = swerveConstants;
        this.JOYSTICK_CONSTANTS = joystickConstants;
        this.MODULE_CONSTANTS = moduleConstants;
        this.DRIVE_POSITION_TOLERANCE = driveTolerances.getFirst();
        this.DRIVE_VELOCITY_TOLERANCE = driveTolerances.getSecond();
        this.ROTATION_POSITION_TOLERANCE = rotationTolerances.getFirst();
        this.ROTATION_VELOCITY_TOLERANCE = rotationTolerances.getSecond();
        this.ODOMETRY_THREAD_PRIORITY = odometryThreadPriority;
        this.PHASE_DELAY = phaseDelay;
        this.ROBOT_MASS = mass;
        this.ROBOT_MOI = moi;
        this.TRACK_WIDTH = dimensions.getFirst();
        this.WHEEL_BASE = dimensions.getSecond();
        this.THETA_MAX_VELOCITY = thetaLimits.getFirst();
        this.THETA_MAX_ACCELERATION = thetaLimits.getSecond();
        this.THETA_PID = thetaPid;
        this.WHEEL_COF = wheelCof;
        this.WHEEL_RADIUS = wheelRadius;
        this.XY_FF_MIN_RANGE = feedforwardRange.getFirst();
        this.XY_FF_MAX_RANGE = feedforwardRange.getSecond();
        this.XY_PID = robotPID;
            
        this.modules = KrakenCoaxialSwerveModule.ofFinishedDrivetrain(this);
        this.SWERVE_MODULE_LOCATIONS = new Translation2d[] {
                new Translation2d(WHEEL_BASE.div(2.0), TRACK_WIDTH.div(2.0)),
                new Translation2d(WHEEL_BASE.div(2.0), TRACK_WIDTH.div(2.0).unaryMinus()),
                new Translation2d(WHEEL_BASE.div(2.0).unaryMinus(), TRACK_WIDTH.div(2.0)),
                new Translation2d(WHEEL_BASE.div(2.0).unaryMinus(), TRACK_WIDTH.div(2.0).unaryMinus()) };
        this.pigeon = new Pigeon2(PIGEON_ID, CAN_BUS);
        this.DRIVE_BASE_RADIUS = Meters
                .of(Math.hypot(TRACK_WIDTH.div(2.0).in(Meters), WHEEL_BASE.div(2.0).in(Meters)));
        this.ROBOT_CONFIG = new RobotConfig(ROBOT_MASS, ROBOT_MOI,
                new ModuleConfig(WHEEL_RADIUS, SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY, WHEEL_COF,
                        SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR, SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT, 1),
                SWERVE_MODULE_LOCATIONS);
        this.rotationController = new ProfiledPIDController(
                THETA_PID.getkP(), THETA_PID.getkI(), THETA_PID.getkD(),
                new TrapezoidProfile.Constraints(THETA_MAX_VELOCITY.in(RadiansPerSecond),
                        THETA_MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));
        this.driveController = new ProfiledPIDController(XY_PID.getkP(), XY_PID.getkI(), XY_PID.getkD(),
                new TrapezoidProfile.Constraints(SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY.in(MetersPerSecond),
                        SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION.in(MetersPerSecondPerSecond)));
        this.KINEMATICS = new SwerveDriveKinematics(SWERVE_MODULE_LOCATIONS[0],
                SWERVE_MODULE_LOCATIONS[1], SWERVE_MODULE_LOCATIONS[2], SWERVE_MODULE_LOCATIONS[3]);
        this.swerve =  new KrakenSwerveDrive(modules[0], modules[1], modules[2], modules[3], pigeon,
            driveMode, KINEMATICS, SWERVE_CONSTANTS, this, DRIVE_SYSID_CONFIG, STEER_SYSID_CONFIG,
            ODOMETRY_THREAD_PRIORITY);

        rotationController.setTolerance(ROTATION_POSITION_TOLERANCE.in(Radians),
                ROTATION_VELOCITY_TOLERANCE.in(RadiansPerSecond));
        rotationController.enableContinuousInput(0.0, 2.0 * Math.PI);

        driveController.setTolerance(DRIVE_POSITION_TOLERANCE.in(Meters), DRIVE_VELOCITY_TOLERANCE.in(MetersPerSecond));
        Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
        pigeonConfig.MountPose.withMountPosePitch(pigeonPitchYawRoll.get(0));
        pigeonConfig.MountPose.withMountPoseYaw(pigeonPitchYawRoll.get(1));
        pigeonConfig.MountPose.withMountPoseRoll(pigeonPitchYawRoll.get(2));
        pigeon.getConfigurator().apply(pigeonConfig);
    }

    public FinishedDrivetrain(CANBus bus, ModuleConstants[] moduleConstants, Mass robotMass, MomentOfInertia robotMoi, SwerveConstants swerveConstants){
        this(bus,
            swerveConstants,
            new JoystickConstants(3.0, 0.1, 2.0, 2.0),
            moduleConstants,
            new Pair<>(Inches.of(8.0), MetersPerSecond.of(0.5)),
            new Pair<>(Degrees.of(2.5), DegreesPerSecond.of(0.5)),
            1,
            Seconds.of(0.3),
            robotMass,
            robotMoi,
            false,
            false,
            false,
            new Pair<>(Inches.of(24.75), Inches.of(19.25)),
            new Pair<>(RadiansPerSecond.of(10.0), RadiansPerSecondPerSecond.of(15.0)),
            new TuningConstants(8.0, 0.0, 0.1),
            1.3,
            swerveConstants.WHEEL_CIRCUMFERENCE.div(2 * Math.PI),
            new Pair<>(0.1, 0.15),
            new TuningConstants(8.0, 0.0, 0.05),
            VecBuilder.fill(1.0, 2.0, 5.2)
        );
    }

    /** Draw the robot on a Field2d every 20ms */
    @Override
    public void periodic() {
        swerve.drawRobotOnField(AutoManager.getInstance().getField());

        lastFieldRelativeChassisSpeeds = swerve.getFieldRelativeSpeeds();
    }

    /** Update simulation specific variables every 20ms */
    @Override
    public void simulationPeriodic() {
        SwerveModulePosition[] currentPositions = swerve.getModulePositions();
        SwerveModulePosition[] deltas = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            deltas[i] = new SwerveModulePosition(
                    currentPositions[i].distanceMeters - lastModulePositions[i].distanceMeters,
                    currentPositions[i].angle);
        }

        pigeon.getSimState().setRawYaw(pigeon.getYaw().getValueAsDouble() +
                Units.radiansToDegrees(KINEMATICS.toTwist2d(deltas).dtheta));

        lastModulePositions = currentPositions;
    }

    public Command teleopDriveCommand(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
            DoubleSupplier thetaSpeedSupplier) {

        // Initiate speed limiter ojects from SlewRateLimiter
        SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(JOYSTICK_CONSTANTS.INPUT_RATE_LIMIT);
        SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(JOYSTICK_CONSTANTS.INPUT_RATE_LIMIT);
        SlewRateLimiter thetaSpeedLimiter = new SlewRateLimiter(JOYSTICK_CONSTANTS.INPUT_RATE_LIMIT);

        // Add a lambda making local variables related to the values
        return run(() -> {
            double xSpeed = xSpeedSupplier.getAsDouble();
            double ySpeed = ySpeedSupplier.getAsDouble();
            double thetaSpeed = thetaSpeedSupplier.getAsDouble();

            xSpeed = MathUtil.applyDeadband(xSpeed, JOYSTICK_CONSTANTS.INPUT_DEADBAND);
            ySpeed = MathUtil.applyDeadband(ySpeed, JOYSTICK_CONSTANTS.INPUT_DEADBAND);
            thetaSpeed = MathUtil.applyDeadband(thetaSpeed, JOYSTICK_CONSTANTS.INPUT_DEADBAND);

            xSpeed = Math.copySign(Math.pow(xSpeed, JOYSTICK_CONSTANTS.CURVE_EXP), xSpeed);
            ySpeed = Math.copySign(Math.pow(ySpeed, JOYSTICK_CONSTANTS.CURVE_EXP), ySpeed);
            thetaSpeed = Math.copySign(Math.pow(thetaSpeed, JOYSTICK_CONSTANTS.ROT_CURVE_EXP), thetaSpeed);

            xSpeed = xSpeedLimiter.calculate(xSpeed);
            ySpeed = ySpeedLimiter.calculate(ySpeed);
            thetaSpeed = thetaSpeedLimiter.calculate(thetaSpeed);

            // Values will show as between -1.0 and 1.0, so multiply by max driving velocity
            // so values will be in m/s
            xSpeed *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY.in(MetersPerSecond);
            ySpeed *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY.in(MetersPerSecond);
            thetaSpeed *= THETA_MAX_VELOCITY.in(RadiansPerSecond);

            if (isControlledRotationEnabled) {
                if (rotationController.atGoal()) {
                    isControlledRotationEnabled = false;
                } else {
                    thetaSpeed = rotationController.calculate(swerve.getRotation().getRadians());
                }
            }

            swerve.drive(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), driveMode);
        }).withName("drivetrain.teleopDrive");
    }

    public Command controlledRotateCommand(Supplier<Angle> angle) {
        return Commands.runOnce(() -> {
            if (!isControlledRotationEnabled) {
                rotationController.reset(swerve.getRotation().getRadians());
            }
            isControlledRotationEnabled = true;
            if (Utils.isRobotRedAlliance()) // TODO check if this is right, i think method changed from old method
                rotationController.setGoal(angle.get().in(Radians) + Math.PI);
            else
                rotationController.setGoal(angle.get().in(Radians));
        }).withName("drivetrain.controlledRotateCommand");
    }

    public Command disableControlledRotateCommand() {
        return Commands.runOnce(() -> {
            isControlledRotationEnabled = false;
        }).withName("drivetrain.disableControlledRotateCommand");
    }

    public Command wheelLockCommand() {
        return run(() -> {
            swerve.setStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            });
        }).withName("drivetrain.wheelLock");
    }

    public Command turnWheelsToAngleCommand(Angle angle) {
        return runOnce(() -> {
            swerve.setStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle))
            });
        }).withName("drivetrain.turnWheelsToAngle");
    }

    public Command driveToPoseCommand(Supplier<Pose2d> poseSupplier) {
        return Commands.runOnce(() -> {
            Translation2d currentPosition = swerve.getPrecisePose().getTranslation();
            Translation2d targetPosition = poseSupplier.get().getTranslation();

            Translation2d toTarget = targetPosition.minus(currentPosition);
            Rotation2d directionToTarget = toTarget.getAngle();

            ChassisSpeeds currentSpeeds = swerve.getFieldRelativeSpeeds();
            Translation2d currentVelocity = new Translation2d(currentSpeeds.vxMetersPerSecond,
                    currentSpeeds.vyMetersPerSecond);

            double velocityTarget = -currentVelocity.getNorm() * Math.cos(
                    currentVelocity.getAngle().minus(directionToTarget).getRadians());

            driveController.reset(toTarget.getNorm(), velocityTarget < 0 ? velocityTarget : 0);
            rotationController.reset(swerve.getPrecisePose().getRotation().getRadians());
        }).andThen(run(() -> {
            driveController.reset(
                    swerve.getPrecisePose().getTranslation().getDistance(poseSupplier.get().getTranslation()),
                    driveController.getSetpoint().velocity);
            // using one controller (distance from pose) compared to two (x and y distance)
            // so that we move in a straight line and not a curve
            driveToPoseDistance = swerve.getPrecisePose().getTranslation()
                    .getDistance(poseSupplier.get().getTranslation());

            // prevents fast swapping of feedforward back and forth around setpoint
            double ffScaler = MathUtil.clamp(
                    (driveToPoseDistance - XY_FF_MIN_RANGE) / (XY_FF_MAX_RANGE - XY_FF_MIN_RANGE),
                    0.0,
                    1.0);

            driveToPoseScalar = driveController.getSetpoint().velocity * ffScaler + driveController.calculate(
                    driveToPoseDistance, 0.0);
            if (Utils.isRobotRedAlliance()) { // TODOD check this aswell
                driveToPoseScalar *= -1;
            }
            if (driveController.atGoal())
                driveToPoseScalar = 0.0;

            double thetaVelocity = rotationController.getSetpoint().velocity * ffScaler
                    + rotationController.calculate(swerve.getPrecisePose().getRotation().getRadians(),
                            poseSupplier.get().getRotation().getRadians());

            Translation2d driveVelocity = new Pose2d(
                    new Translation2d(),
                    swerve.getPrecisePose().getTranslation().minus(poseSupplier.get().getTranslation()).getAngle())
                    .transformBy(new Transform2d(driveToPoseScalar, 0, Rotation2d.kZero))
                    .getTranslation();

            swerve.driveClosedLoop(
                    new ChassisSpeeds(
                            driveVelocity.getX(),
                            driveVelocity.getY(),
                            thetaVelocity),
                    DriveMode.FIELD_ORIENTED);
        })).withName("drivetrain.driveToPose");
    }

    /**
     * Creates a command that drives the robot chassis to a specific pose. Will
     * self-cancel upon reaching the desired pose.
     * 
     * @param poseSupplier a supplier of the desired pose
     * @return the command
     */
    public Command driveToPoseUntilAtGoalCommand(Supplier<Pose2d> poseSupplier) {
        return driveToPoseCommand(poseSupplier)
                .until(() -> Meters.of(poseSupplier.get().minus(swerve.getPrecisePose()).getTranslation().getNorm())
                        .isNear(Meters.zero(), DRIVE_POSITION_TOLERANCE))
                .withName("drivetrain.driveToPoseUntilAtGoal");
    }

    public Command followPathCommand(PathPlannerPath path) {
        return new FollowPathCommand(path, swerve::getPrecisePose, swerve::getChassisSpeeds, swerve::driveClosedLoop,
                new PPHolonomicDriveController(new PIDConstants(PATH_FOLLOWING_TRANSLATION_kP),
                        new PIDConstants(PATH_FOLLOWING_ROTATION_kP)),
                ROBOT_CONFIG, () -> Utils.isRobotRedAlliance(), this) // TODO check this aswell
                .andThen(runOnce(() -> swerve.stop())).withName("drivetrain.followPathCommand");
    }

    public Command driveDeltaCommand(Transform2d delta, PathConstraints constraints) {
        return new DeferredCommand(() -> followPathCommand(
                new PathPlannerPath(PathPlannerPath.waypointsFromPoses(swerve.getPose(), swerve.getPose().plus(delta)),
                        constraints, new IdealStartingState(0.0, swerve.getRotation()),
                        new GoalEndState(0.0, swerve.getRotation().plus(delta.getRotation())))),
                Set.of(this)).withName("drivetrain.driveDelta");
    }

    public Command setDriveModeCommand(DriveMode driveMode) {
        return Commands.runOnce(() -> {
            this.driveMode = driveMode;
        }).withName("drivetrain.setDriveMode");
    }

    public Command resetGyroCommand() {
        return runOnce(() -> {
            pigeon.setYaw(0);
            initialized = true;
        });
    }

    public Command setDriveCurrentLimitCommand(Current currentLimit) {
        return Commands.runOnce(() -> {
            swerve.setDriveCurrentLimits((int) currentLimit.in(Amps));
        }).withName("drivetrain.setDriveCurrentLimit");
    }

    public Command coastMotorsCommand() {
        return startEnd(() -> {
            swerve.stop();
            swerve.setMotorNeutralModes(NeutralModeValue.Coast);
        }, () -> {
            swerve.setMotorNeutralModes(NeutralModeValue.Brake);
        }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("drivetrain.coastMotors");
    }

    public Command sysIdDriveQuasistaticCommand(Direction direction) {
        return swerve.getSysIdDrive().quasistatic(direction).withName("drivetrain.sysIdDriveQuasistatic");
    }

    public Command sysIdDriveDynamicCommand(Direction direction) {
        return swerve.getSysIdDrive().dynamic(direction).withName("drivetrain.sysIdDriveDynamic");
    }

    public Command sysIdSteerQuasistaticCommand(Direction direction) {
        return swerve.getSysIdSteer().quasistatic(direction).withName("drivetrain.sysIdSteerQuasistatic");
    }

    public Command sysIdSteerDynamicCommand(Direction direction) {
        return swerve.getSysIdSteer().dynamic(direction).withName("drivetrain.sysIdSteerDynamic");
    }

    /**
     * Get the current field-relative accelerations of the robot
     * 
     * @return the accelerations
     */
    public ChassisAccelerations getFieldRelativeAccelerations() {
        return new ChassisAccelerations(swerve.getFieldRelativeSpeeds(), lastFieldRelativeChassisSpeeds, 0.02);
    }

    /**
     * Creates a command that has the robot stay on a line described by a pose on
     * the line and the direction it faces
     * 
     * @param poseSupplier              a supplier of the pose on the line
     * @param rotationTransformSupplier a supplier of the way the robot should face
     *                                  while on the line
     * @param xJoystickSupplier         a supplier of joystick x-axis input
     * @param yJoystickSupplier         a supplier of joystick y-axis input
     * @return
     */
    public Command lockToLineCommand(Supplier<Pose2d> poseSupplier, Supplier<Rotation2d> rotationTransformSupplier,
            DoubleSupplier xJoystickSupplier, DoubleSupplier yJoystickSupplier) {
        SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(JOYSTICK_CONSTANTS.INPUT_RATE_LIMIT);
        SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(JOYSTICK_CONSTANTS.INPUT_RATE_LIMIT);

        return Commands.runOnce(() -> {
            Translation2d currentPos = swerve.getPrecisePose().getTranslation();
            Pose2d closestPoseOnLine = Utils.getClosestPoseOnLine(swerve.getPrecisePose(), poseSupplier.get());
            Translation2d closestPos = closestPoseOnLine.getTranslation();

            Rotation2d lineDirection = poseSupplier.get().getRotation();

            Translation2d toLine = closestPos.minus(currentPos);

            Translation2d perpToLine = new Translation2d(-lineDirection.getSin(), lineDirection.getCos());

            double signedPerpendicularDistance = toLine.getX() * perpToLine.getX() + toLine.getY() * perpToLine.getY();

            Translation2d fieldRelativeVelocity = new Translation2d(
                    swerve.getFieldRelativeSpeeds().vxMetersPerSecond,
                    swerve.getFieldRelativeSpeeds().vyMetersPerSecond);

            double signedPerpendicularVelocity = fieldRelativeVelocity.getX() * perpToLine.getX() +
                    fieldRelativeVelocity.getY() * perpToLine.getY();

            driveController.reset(-signedPerpendicularDistance,
                    signedPerpendicularDistance < 0 ? signedPerpendicularVelocity : 0);
            rotationController.reset(swerve.getPrecisePose().getRotation().getRadians());
        }).andThen(run(() -> {
            Pose2d pose = poseSupplier.get();
            double cosTheta = pose.getRotation().getCos();
            double sinTheta = pose.getRotation().getSin();

            // formula for distance between point and line given Ax + By + C = 0
            driveToPoseDistance = Utils.getLineDistance(swerve.getPrecisePose(), pose);

            double ySpeedRelStage = driveController.calculate(driveToPoseDistance, 0.0)
                    + driveController.getSetpoint().velocity;

            double xJoystick = xJoystickSupplier.getAsDouble();
            xJoystick = MathUtil.applyDeadband(xJoystick, JOYSTICK_CONSTANTS.INPUT_DEADBAND);
            xJoystick = Math.copySign(Math.pow(xJoystick, JOYSTICK_CONSTANTS.CURVE_EXP), xJoystick);
            xJoystick = xSpeedLimiter.calculate(xJoystick);
            xJoystick *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY.in(MetersPerSecond);
            if (Utils.isRobotRedAlliance()) {
                xJoystick *= -1;
            }

            double yJoystick = yJoystickSupplier.getAsDouble();
            yJoystick = MathUtil.applyDeadband(yJoystick, JOYSTICK_CONSTANTS.INPUT_DEADBAND);
            yJoystick = Math.copySign(Math.pow(yJoystick, JOYSTICK_CONSTANTS.CURVE_EXP), yJoystick);
            yJoystick = ySpeedLimiter.calculate(yJoystick);
            yJoystick *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY.in(MetersPerSecond);
            if (Utils.isRobotRedAlliance()) {
                yJoystick *= -1;
            }

            // transforms the vector created by the joystick input to the line created by
            // extending the ray from the trap to the edge of the field
            double lineDirX = Math.cos(pose.getRotation().getRadians());
            double lineDirY = Math.sin(pose.getRotation().getRadians());

            double xJoystickSpeedRelStage = xJoystick * lineDirX + yJoystick * lineDirY;

            double xSpeed = xJoystickSpeedRelStage * cosTheta - ySpeedRelStage * sinTheta;
            double ySpeed = xJoystickSpeedRelStage * sinTheta + ySpeedRelStage * cosTheta;

            if (Utils.isRobotRedAlliance()) {
                xSpeed *= -1;
                ySpeed *= -1;
            }

            double thetaSpeed = rotationController.calculate(swerve.getRotation().getRadians(),
                    pose.getRotation().plus(rotationTransformSupplier.get()).getRadians());

            swerve.driveClosedLoop(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), DriveMode.FIELD_ORIENTED);
        })).withName("drivetrain.lockToLine");
    }

    /**
     * Gets the pose of the drivetrain adjusted for the robot's phase delay.
     * Idea credit to 6328 Mechanical Advantage
     * 
     * @return the adjusted pose
     */
    public Pose2d getPoseAdjustedForPhaseDelay() {
        return swerve.getPose().exp(new Twist2d(
                swerve.getChassisSpeeds().vxMetersPerSecond * PHASE_DELAY.in(Seconds),
                swerve.getChassisSpeeds().vyMetersPerSecond * PHASE_DELAY.in(Seconds),
                swerve.getChassisSpeeds().omegaRadiansPerSecond * PHASE_DELAY.in(Seconds)));
    }

    /**
     * Creates a command that has the robot rotate to the closest 45 degree angle
     * 
     * @return the command
     */
    public Command snapToNearestDiagonalCommand() {
        return controlledRotateCommand(() -> {
            double currentAngleDegrees = MathUtil.inputModulus(swerve.getRawYaw().in(Degrees), 0.0, 360.0);
            if (currentAngleDegrees > 270.0) {
                return Degrees.of(315.0);
            } else if (currentAngleDegrees > 180.0) {
                return Degrees.of(225.0);
            } else if (currentAngleDegrees > 90.0) {
                return Degrees.of(135.0);
            } else {
                return Degrees.of(45.0);
            } // TODO doesn't currently go to nearest
        }).withName("drivetrain.snapToNearestDiagonal");
    }

    /**
     * A method to return whether the drivetrain has been zeroed
     * 
     * @return true if yes, false otherwise
     */
    public boolean getInitialized() {
        return initialized;
    }
}
