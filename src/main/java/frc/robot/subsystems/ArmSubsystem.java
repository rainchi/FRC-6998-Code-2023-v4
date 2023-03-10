package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this ArmSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static ArmSubsystem INSTANCE = new ArmSubsystem();

    /**
     * Returns the Singleton instance of this ArmSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code ArmSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

    private final CANSparkMax topArmMotor = new CANSparkMax(Constants.ARM_TOP_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax bottomArmMotor = new CANSparkMax(Constants.ARM_BOTTOM_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final Solenoid armSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.ARM_SOLENOID_ID);
    private final AbsoluteEncoder topDutyCycleEncoder = topArmMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private final AbsoluteEncoder bottomDutyCycleEncoder = bottomArmMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private double angleSetpoint = 0;

    /**
     * Creates a new instance of this ArmSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ArmSubsystem() {
        // factory default values
        topArmMotor.restoreFactoryDefaults();
        bottomArmMotor.restoreFactoryDefaults();

        // set up current limiting
        topArmMotor.setSmartCurrentLimit(35);
        bottomArmMotor.setSmartCurrentLimit(35);

        topArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 130);
        topArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

        topArmMotor.getPIDController().setFF(Constants.ARM_TOP_KF);
        topArmMotor.getPIDController().setP(Constants.ARM_TOP_KP);
        topArmMotor.getPIDController().setSmartMotionMaxVelocity(Constants.ARM_TOP_MAX_VELOCITY_DEG_PER_SEC, 0);
        topArmMotor.getPIDController().setSmartMotionMaxAccel(Constants.ARM_TOP_MAX_ACCELERATION_DEG_PER_SQ, 0);
        topArmMotor.getPIDController().setSmartMotionAllowedClosedLoopError(Constants.ARM_TOP_MAX_ALLOWABLE_ERROR_DEG, 0);
        bottomArmMotor.getPIDController().setFF(Constants.ARM_BOTTOM_KF);
        bottomArmMotor.getPIDController().setP(Constants.ARM_BOTTOM_KP);
        bottomArmMotor.getPIDController().setSmartMotionMaxVelocity(Constants.ARM_BOTTOM_MAX_VELOCITY_DEG_PER_SEC, 0);
        bottomArmMotor.getPIDController().setSmartMotionMaxAccel(Constants.ARM_BOTTOM_MAX_ACCELERATION_DEG_PER_SQ, 0);
        bottomArmMotor.getPIDController().setSmartMotionAllowedClosedLoopError(Constants.ARM_BOTTOM_MAX_ALLOWABLE_ERROR_DEG, 0);

        // convert encoder position to degrees
        topArmMotor.getEncoder().setPositionConversionFactor(1/Constants.ARM_TOP_GEARING*360); // round to degrees
        topArmMotor.getEncoder().setVelocityConversionFactor(1/Constants.ARM_TOP_GEARING*360/60); // rpm to degrees per second
        bottomArmMotor.getEncoder().setPositionConversionFactor(1/ Constants.ARM_BOTTOM_GEARING*360); // round to degrees
        bottomArmMotor.getEncoder().setVelocityConversionFactor(1/ Constants.ARM_BOTTOM_GEARING*360/60); // rpm to degrees per second


        topDutyCycleEncoder.setPositionConversionFactor(360);
        topDutyCycleEncoder.setVelocityConversionFactor(360/60.0);
        topDutyCycleEncoder.setZeroOffset(Constants.ARM_TOP_ENCODER_OFFSET%360);

        bottomDutyCycleEncoder.setPositionConversionFactor(360);
        bottomDutyCycleEncoder.setVelocityConversionFactor(360/60.0);
        bottomDutyCycleEncoder.setZeroOffset(Constants.ARM_BOTTOM_ENCODER_OFFSET%360);

        syncPosition();
    }

    @Override
    public void periodic() {
        //bottomArmMotor.getPIDController().setReference(0, CANSparkMax.ControlType.kVelocity);
        SmartDashboard.putNumber("a-angle", topArmMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("b-angle", bottomArmMotor.getEncoder().getPosition());
    }

    public double getTopArmPositionInRadians() {
        return topArmMotor.getEncoder().getPosition();
    }

    public double getBottomArmPositionInRadians() {
        return bottomArmMotor.getEncoder().getPosition();
    }

    public void raiseArm(){
        angleSetpoint+=10;
        if (angleSetpoint>130) angleSetpoint = 130;
        topArmMotor.getPIDController().setReference(angleSetpoint, CANSparkMax.ControlType.kSmartMotion);
    }

    public void lowerArm(){
        angleSetpoint-=10;
        if (angleSetpoint<0) angleSetpoint = 0;
        topArmMotor.getPIDController().setReference(angleSetpoint, CANSparkMax.ControlType.kSmartMotion);
    }

    public void testRaise(){
        bottomArmMotor.getPIDController().setReference(30, CANSparkMax.ControlType.kSmartMotion);
    }

    public void testLower(){
        bottomArmMotor.getPIDController().setReference(0, CANSparkMax.ControlType.kSmartMotion);
    }

    public void closeClaw() {
        armSolenoid.set(false);
    }

    public void openClaw() {
        armSolenoid.set(true);
    }

    public void syncPosition(){
        double pos = topDutyCycleEncoder.getPosition();
        if (pos>=180) pos-=360;
        topArmMotor.getEncoder().setPosition(pos);
        pos = bottomDutyCycleEncoder.getPosition();
        if (pos>=180) pos-=360;
        bottomArmMotor.getEncoder().setPosition(-pos);
    }
}

