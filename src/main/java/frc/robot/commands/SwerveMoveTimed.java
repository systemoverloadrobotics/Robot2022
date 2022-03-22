package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

class SwerveMoveTimed extends CommandBase {
  private Swerve swerve;
  private int msTime;
  private double x;
  private double y;
  private double rot;

  public SwerveMoveTimed(Swerve swerve, int msTime, double x, double y, double rot) {
    this.swerve = swerve;
    this.msTime = msTime;
    this.x = x;
    this.y = y;
    this.rot = rot;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
  }
}