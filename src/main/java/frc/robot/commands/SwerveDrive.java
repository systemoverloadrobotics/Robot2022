package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveDrive extends CommandBase {

  private final Swerve swerve;
  private DoubleSupplier xSupplier, ySupplier, rotationSupplier;
  private SlewRateLimiter xLimiter, yLimiter, rotationLimiter;// if the joystick is moved violently

  public SwerveDrive(Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
    this.swerve = swerve;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;
    xLimiter = new SlewRateLimiter(Constants.Motor.SWERVE_MAX_SPEED);
    yLimiter = new SlewRateLimiter(Constants.Motor.SWERVE_MAX_SPEED);
    rotationLimiter = new SlewRateLimiter(Constants.Motor.SWERVE_MAX_SPEED);
    addRequirements(swerve);
  }

  // Called when the command is first scheduled.
  @Override
  public void initialize() {
    
  }

  // Called at 50hz while the command is scheduled.
  @Override
  public void execute() {
    //get joystick inputs
    double x1speed = xSupplier.getAsDouble();
    double y1speed = ySupplier.getAsDouble();
    double x2speed = rotationSupplier.getAsDouble();
    
    //apply deadband
    x1speed = Math.abs(x1speed) > Constants.Motor.SWERVE_DEADBAND ? x1speed : 0.0;
    y1speed = Math.abs(y1speed) > Constants.Motor.SWERVE_DEADBAND ? y1speed : 0.0;
    x2speed = Math.abs(x2speed) > Constants.Motor.SWERVE_DEADBAND ? x2speed : 0.0;

    //smooth driving
    x1speed = xLimiter.calculate(x1speed) * Constants.Motor.SWERVE_MAX_SPEED;
    y1speed = yLimiter.calculate(y1speed) * Constants.Motor.SWERVE_MAX_SPEED;
    x2speed = rotationLimiter.calculate(x2speed) * Constants.Motor.SWERVE_MAX_SPEED;

    //construct chassis
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      x1speed, y1speed, x2speed, swerve.getRotation2d());

    //convert to states from the chassis  
    SwerveModuleState[] moduleState = Constants.Motor.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    swerve.setModuleStates(moduleState);
  }

  // Called once when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
