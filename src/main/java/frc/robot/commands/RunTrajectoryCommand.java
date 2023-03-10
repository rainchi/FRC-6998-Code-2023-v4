package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;


public class RunTrajectoryCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final Trajectory trajectory;
    private final HolonomicDriveController follower;
    private final SwerveDriveKinematics kinematics;
    private final DriveSubsystem drive;
    private final Rotation2d targetRot;

    public RunTrajectoryCommand(DriveSubsystem driveTrainSubsystem, Trajectory trajectory) {
        this.trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
        List<Trajectory.State> states = trajectory.getStates();
        this.targetRot = states.get(states.size()-1).poseMeters.getRotation();
        follower = new HolonomicDriveController(
                new PIDController(1,0,0),
                new PIDController(1,0,0),
                new ProfiledPIDController(1,0,0, new TrapezoidProfile.Constraints(6.28, 3.14)));
        kinematics = DriveSubsystem.getInstance().getKinematics();
        drive = driveTrainSubsystem;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetPose(trajectory.getInitialPose());
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double curTime = timer.get();

        var targetWheelSpeeds =
                kinematics.toSwerveModuleStates(
                        follower.calculate(drive.getPose(), trajectory.sample(curTime), targetRot));

        drive.setModuleStates(targetWheelSpeeds);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drive.stopAll();
    }
}