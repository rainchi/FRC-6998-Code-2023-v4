package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
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

    /**
     * Creates a new instance of this ArmSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ArmSubsystem() {
        // factory default values
        topArmMotor.restoreFactoryDefaults();
        bottomArmMotor.restoreFactoryDefaults();

        topArmMotor.setSmartCurrentLimit(40);
        bottomArmMotor.setSmartCurrentLimit(40);

        // convert encoder position to radians
        double factor = 1/Constants.ARM_GEARING*Math.PI;
        topArmMotor.getEncoder().setPositionConversionFactor(factor);
        bottomArmMotor.getEncoder().setPositionConversionFactor(factor);
    }

    public double getTopArmPositionInRadians() {
        return topArmMotor.getEncoder().getPosition();
    }

    public double getBottomArmPositionInRadians() {
        return bottomArmMotor.getEncoder().getPosition();
    }
}

