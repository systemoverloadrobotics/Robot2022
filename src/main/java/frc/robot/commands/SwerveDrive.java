package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveDrive extends CommandBase {

  private final Swerve swerve;
  private DoubleSupplier x1Supplier, y1Supplier, x2Supplier;
  private SlewRateLimiter x1Limiter, y1Limiter, x2Limiter;// if the joystick is moved violently

  public SwerveDrive(Swerve swerve, DoubleSupplier x1Supplier, DoubleSupplier y1Supplier, DoubleSupplier x2Supplier) {
    this.swerve = swerve;
    this.x1Supplier = x1Supplier;
    this.y1Supplier = y1Supplier;
    this.x2Supplier = x2Supplier;
    x1Limiter = new SlewRateLimiter(Constants.Motor.SWERVE_MAX_SPEED);
    y1Limiter = new SlewRateLimiter(Constants.Motor.SWERVE_MAX_SPEED);
    x2Limiter = new SlewRateLimiter(Constants.Motor.SWERVE_MAX_SPEED);
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
    double x1speed = x1Supplier.getAsDouble();
    double y1speed = y1Supplier.getAsDouble();
    double x2speed = x2Supplier.getAsDouble();
    
    //apply deadband
    x1speed = Math.abs(x1speed) > Constants.Motor.SWERVE_DEADBAND ? x1speed : 0.0;
    y1speed = Math.abs(y1speed) > Constants.Motor.SWERVE_DEADBAND ? y1speed : 0.0;
    x2speed = Math.abs(x2speed) > Constants.Motor.SWERVE_DEADBAND ? x2speed : 0.0;

    //smooth driving
    x1speed = x1Limiter.calculate(x1speed) * Constants.Motor.SWERVE_MAX_SPEED;
    y1speed = y1Limiter.calculate(y1speed) * Constants.Motor.SWERVE_MAX_SPEED;
    x2speed = x2Limiter.calculate(x2speed) * Constants.Motor.SWERVE_MAX_SPEED;

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
