// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);

    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
//    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();

    private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            driveSubsystem::getPose, // Pose2d supplier
            driveSubsystem::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
            driveSubsystem.getKinematics(), // SwerveDriveKinematics
            new PIDConstants(1, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.1, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            driveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
            null, // The path constraints. Optional, defaults to null
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
    );

    PathPlannerTrajectory trajectory = PathPlanner.loadPath("Path 1", new PathConstraints(Constants.CHASSIS_MAX_SPEED_METERS_PER_SECOND, Constants.CHASSIS_MAX_ACCELERATION_METERS_PER_SECOND));

    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();
        SwerveDriveCommand driveCommand = new SwerveDriveCommand(driverController);
        driveSubsystem.setDefaultCommand(driveCommand);
    }
    
    
    /** Use this method to define your trigger->command mappings. */
    private void configureBindings()
    {
//        new JoystickButton(driverController, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(armSubsystem::raiseArm));
//        new JoystickButton(driverController, XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(armSubsystem::lowerArm));
//        new JoystickButton(driverController, XboxController.Button.kA.value).onTrue(new InstantCommand(armSubsystem::closeClaw));
//        new JoystickButton(driverController, XboxController.Button.kY.value).onTrue(new InstantCommand(armSubsystem::openClaw));
//        new JoystickButton(driverController, XboxController.Button.kX.value).onTrue(new InstantCommand(armSubsystem::testLower));
//        new JoystickButton(driverController, XboxController.Button.kB.value).onTrue(new InstantCommand(armSubsystem::testRaise));
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return autoBuilder.fullAuto(trajectory).andThen(new AutoBalanceCommand(false));
        //return new AutoBalanceCommand(false);
    }
}
