package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;


public class AutoBalanceCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    private final PIDController pidController = new PIDController(Constants.AUTO_BALANCE_KP,Constants.AUTO_BALANCE_KI,Constants.AUTO_BALANCE_KD);
    private final Timer timer = new Timer();
    private Stage currentStage;
    private final boolean shouldExitWhenFinished;

    public AutoBalanceCommand(boolean shouldExitWhenFinished) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveSubsystem);
        currentStage = Stage.Created;
        this.shouldExitWhenFinished = shouldExitWhenFinished;
    }

    @Override
    public void initialize() {
        currentStage = Stage.Preparing;
        timer.start();
    }

    @Override
    public void execute() {
        double error = -driveSubsystem.navX.getPitch();
        switch (currentStage) {
            case Preparing -> {
                driveSubsystem.drive(0, Constants.AUTO_BALANCE_START_SPEED_METERS_PER_SECOND, 0, false);
                if (Math.abs(error) > Constants.AUTO_BALANCE_TOLERANCE) {
                    currentStage = Stage.Climb;
                    timer.reset();
                }
            }
            case Climb -> {
                driveSubsystem.drive(0, pidController.calculate(error), 0, false);
                if (Math.abs(error) <= Constants.AUTO_BALANCE_TOLERANCE) {
                    currentStage = Stage.Wait;
                    timer.reset();
                }
            }
            case Wait -> {
                driveSubsystem.drive(0,0,0,false);
                if (Math.abs(error) > Constants.AUTO_BALANCE_TOLERANCE) {
                    currentStage = Stage.Climb;
                } else {
                    if (shouldExitWhenFinished && timer.get() > Constants.AUTO_BALANCE_WAIT_TIME) {
                        currentStage = Stage.Finish;
                    }
                }
            }
            default -> {
            }
        }
    }

    @Override
    public boolean isFinished() {
        return shouldExitWhenFinished && currentStage==Stage.Finish;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0,0,0,false);
        timer.stop();
    }

    public enum Stage{
        Created,Preparing,Climb,Wait,Finish
    }
}
