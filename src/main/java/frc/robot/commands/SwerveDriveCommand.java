package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;


public class SwerveDriveCommand extends CommandBase {
    private final DriveSubsystem drive = DriveSubsystem.getInstance();
    private final XboxController driveController;

    public SwerveDriveCommand(XboxController driveController) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(drive);
        this.driveController = driveController;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double xSpeed = MathUtil.applyDeadband(-driveController.getLeftY(), Constants.CHASSIS_DEAD_ZONE);
        double ySpeed = MathUtil.applyDeadband(-driveController.getLeftX(), Constants.CHASSIS_DEAD_ZONE);
        double zRot = MathUtil.applyDeadband(-driveController.getRightX(), Constants.CHASSIS_DEAD_ZONE);
        xSpeed *= Constants.CHASSIS_MAX_SPEED_METERS_PER_SECOND;
        ySpeed *= Constants.CHASSIS_MAX_SPEED_METERS_PER_SECOND;
        zRot *= Constants.CHASSIS_MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        drive.drive(xSpeed, ySpeed, zRot, Constants.CHASSIS_ENABLE_FIELD_ORIENTED_CONTROL);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopAll();
    }
}
