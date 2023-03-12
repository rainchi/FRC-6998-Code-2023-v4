package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this DriveSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static DriveSubsystem INSTANCE = new DriveSubsystem();

    /**
     * Returns the Singleton instance of this DriveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code DriveSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static DriveSubsystem getInstance() {
        return INSTANCE;
    }

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.CHASSIS_LENGTH_METERS / 2.0, Constants.CHASSIS_WIDTH_METERS / 2.0), // Front Left
            new Translation2d(Constants.CHASSIS_LENGTH_METERS / 2.0, -Constants.CHASSIS_WIDTH_METERS / 2.0), // Front Right
            new Translation2d(-Constants.CHASSIS_LENGTH_METERS / 2.0, Constants.CHASSIS_WIDTH_METERS / 2.0), // Rear Left
            new Translation2d(-Constants.CHASSIS_LENGTH_METERS / 2.0, -Constants.CHASSIS_WIDTH_METERS / 2.0) // Rear Right
    );

    private final SwerveDriveOdometry odometry;

    public final AHRS navX = new AHRS(SPI.Port.kMXP); // NavX Sensor

    private final SwerveModule[] swerveModules = new SwerveModule[]{
            new SwerveModule(Constants.CHASSIS_FRONT_LEFT_DRIVE_MOTOR_ID, Constants.CHASSIS_FRONT_LEFT_ANGLE_MOTOR_ID, Constants.CHASSIS_FRONT_LEFT_CANCODER_ID, Constants.CHASSIS_FRONT_LEFT_ANGLE_OFFSET_DEGREES), // Front Left
            new SwerveModule(Constants.CHASSIS_FRONT_RIGHT_DRIVE_MOTOR_ID, Constants.CHASSIS_FRONT_RIGHT_ANGLE_MOTOR_ID, Constants.CHASSIS_FRONT_RIGHT_CANCODER_ID, Constants.CHASSIS_FRONT_RIGHT_ANGLE_OFFSET_DEGREES), // Front Right
            new SwerveModule(Constants.CHASSIS_BACK_LEFT_DRIVE_MOTOR_ID, Constants.CHASSIS_BACK_LEFT_ANGLE_MOTOR_ID, Constants.CHASSIS_BACK_LEFT_CANCODER_ID, Constants.CHASSIS_BACK_LEFT_ANGLE_OFFSET_DEGREES), // Rear Left
            new SwerveModule(Constants.CHASSIS_BACK_RIGHT_DRIVE_MOTOR_ID, Constants.CHASSIS_BACK_RIGHT_ANGLE_MOTOR_ID, Constants.CHASSIS_BACK_RIGHT_CANCODER_ID, Constants.CHASSIS_BACK_RIGHT_ANGLE_OFFSET_DEGREES), // Rear Right
    };

    /**
     * Creates a new instance of this DriveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private DriveSubsystem() {
        navX.zeroYaw();
        odometry = new SwerveDriveOdometry(
                kinematics, navX.getRotation2d(),
                getModulePositions(), Constants.CHASSIS_INITIAL_POSITION);
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(navX.getRotation2d(), getModulePositions(), pose);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
        };
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navX.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rot));
        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    public void syncPositions(){
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.syncPosition();
        }
    }

    public void stopAll(){
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.stop();
        }
    }

    @Override
    public void periodic() {
        odometry.update(navX.getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry Angle", odometry.getPoseMeters().getRotation().getDegrees());
    }

    public static class SwerveModule {
        private final WPI_TalonFX driveMotor;
        private final CANSparkMax angleMotor;
        private final WPI_CANCoder canCoder;
        private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
                Constants.CHASSIS_DRIVE_MOTOR_KS,
                Constants.CHASSIS_DRIVE_MOTOR_KV,
                Constants.CHASSIS_DRIVE_MOTOR_KA
        );

        public SwerveModule(int driveMotorCANID, int angleMotorCANID, int canCoderCANID, double angleOffset) {
            driveMotor = new WPI_TalonFX(driveMotorCANID);
            angleMotor = new CANSparkMax(angleMotorCANID, CANSparkMax.MotorType.kBrushless);
            canCoder = new WPI_CANCoder(canCoderCANID);
            // factory reset drive and angle motors
            driveMotor.configFactoryDefault();
            angleMotor.restoreFactoryDefaults();
            // factory reset canCoder
            canCoder.configFactoryDefault();

            canCoder.setPositionToAbsolute();
            canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
            canCoder.configMagnetOffset(-angleOffset);

            angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            // set pid and feedforward constants
            driveMotor.config_kP(0, Constants.CHASSIS_DRIVE_MOTOR_KP);
            driveMotor.config_kD(0, Constants.CHASSIS_DRIVE_MOTOR_KD);
            double angleRatio = 360 / Constants.CHASSIS_ANGLE_GEAR_RATIO;
            angleMotor.getPIDController().setP(Constants.CHASSIS_ANGLE_MOTOR_KP / angleRatio);
            angleMotor.getPIDController().setD(Constants.CHASSIS_ANGLE_MOTOR_KD / angleRatio);
            angleMotor.getPIDController().setFF(Constants.CHASSIS_ANGLE_MOTOR_KF / angleRatio);
            angleMotor.getPIDController().setSmartMotionMaxVelocity(Constants.CHASSIS_ANGLE_MOTOR_SMART_MOTION_MAX_VELOCITY * angleRatio, 0);
            angleMotor.getPIDController().setSmartMotionMaxAccel(Constants.CHASSIS_ANGLE_MOTOR_SMART_MOTION_MAX_ACCELERATION * angleRatio, 0);
            angleMotor.getPIDController().setSmartMotionAllowedClosedLoopError(Constants.CHASSIS_ANGLE_MOTOR_SMART_MOTION_ALLOWABLE_ERROR * angleRatio, 0);
            // set gear ratio conversion to spark max
            angleMotor.getEncoder().setPositionConversionFactor(angleRatio);
            angleMotor.getEncoder().setVelocityConversionFactor(angleRatio);
            // sync canCoder's position to angleMotor's position
            syncPosition();
        }

        public void syncPosition() {
            angleMotor.getEncoder().setPosition(-canCoder.getAbsolutePosition());
        }

        public final double getDriveGearRatio() {
            return switch (Constants.CHASSIS_DRIVE_GEAR_RATIO) {
                case L1 -> 8.14;
                case L2 -> 6.75;
                case L3 -> 6.12;
                case L4 -> 5.14;
            };
        }

        public void setDesiredState(SwerveModuleState state) {
            state = optimize(state, getRotation());
            driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond* getDriveGearRatio()/10/Constants.CHASSIS_WHEEL_DIAMETER_METERS*2048.0/Math.PI, DemandType.ArbitraryFeedForward, feedforward.calculate(state.speedMetersPerSecond));
//            if (state.speedMetersPerSecond != 0) {
                angleMotor.getPIDController().setReference(-state.angle.getDegrees(), CANSparkMax.ControlType.kSmartMotion);
//            } else {
//                angleMotor.stopMotor();
//            }
        }

        /**
         * Minimize the change in heading the desired swerve module state would require by potentially
         * reversing the direction the wheel spins. Customized from WPILib's version to include placing
         * in appropriate scope for CTRE and REV onboard control.
         *
         * @param desiredState The desired state.
         * @param currentAngle The current module angle.
         */
        public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
            double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
            double targetSpeed = desiredState.speedMetersPerSecond;
            double delta = targetAngle - currentAngle.getDegrees();
            if (Math.abs(delta) > 90){
                targetSpeed = -targetSpeed;
                if (delta > 90) {
                    targetAngle -= 180;
                } else {
                    targetAngle += 180;
                }
            }
            return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
        }

        /**
         * @param scopeReference Current Angle
         * @param newAngle Target Angle
         * @return Closest angle within scope
         */
        private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
            double lowerBound;
            double upperBound;
            double lowerOffset = scopeReference % 360;
            if (lowerOffset >= 0) {
                lowerBound = scopeReference - lowerOffset;
                upperBound = scopeReference + (360 - lowerOffset);
            } else {
                upperBound = scopeReference - lowerOffset;
                lowerBound = scopeReference - (360 + lowerOffset);
            }
            while (newAngle < lowerBound) {
                newAngle += 360;
            }
            while (newAngle > upperBound) {
                newAngle -= 360;
            }
            if (newAngle - scopeReference > 180) {
                newAngle -= 360;
            } else if (newAngle - scopeReference < -180) {
                newAngle += 360;
            }
            return newAngle;
        }

        public final SwerveModulePosition getPosition() {
            return new SwerveModulePosition(
                    driveMotor.getSelectedSensorPosition()/ getDriveGearRatio() / 2048.0 * Math.PI * Constants.CHASSIS_WHEEL_DIAMETER_METERS,
                    getRotation());
        }

        private Rotation2d getRotation(){
            return Rotation2d.fromDegrees(-angleMotor.getEncoder().getPosition());
        }

        public void stop() {
            driveMotor.stopMotor();
            angleMotor.stopMotor();
        }
    }

    public enum GearRatio {
        L1,
        L2,
        L3,
        L4
    }
}

