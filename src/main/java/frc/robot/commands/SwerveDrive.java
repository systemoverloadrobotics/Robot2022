package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveDrive extends CommandBase {
  private final Swerve swerve;

  public SwerveDrive(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is first scheduled.
  @Override
  public void initialize() {
    
  }

  // Called at 50hz while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(Constants.Input.X_AXIS.get().getAsDouble(), Constants.Input.Y_AXIS.get().getAsDouble(), Constants.Input.ROTATION.get().getAsDouble());
  }

  // Called once when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
