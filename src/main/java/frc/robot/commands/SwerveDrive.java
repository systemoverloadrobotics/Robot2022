package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    rotationLimiter = new SlewRateLimiter(Constants.Motor.SWERVE_ROTATION_MAX_SPEED);
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
    double xSpeed = xSupplier.getAsDouble();
    double ySpeed = ySupplier.getAsDouble();
    SmartDashboard.putNumber("XSpeed", xSpeed); 
    SmartDashboard.putNumber("YSpeed", ySpeed); 
    SmartDashboard.putNumber("speed-1", this.swerve.getModuleStates()[0].angle.getDegrees()); 
    SmartDashboard.putNumber("speed-2", this.swerve.getModuleStates()[1].speedMetersPerSecond);   
    SmartDashboard.putNumber("speed-3", this.swerve.getModuleStates()[2].speedMetersPerSecond); 
    SmartDashboard.putNumber("speed-4", this.swerve.getModuleStates()[3].speedMetersPerSecond); 

    double rotationSpeed = rotationSupplier.getAsDouble();
    
    //apply deadband
    xSpeed = Math.abs(xSpeed) > Constants.Motor.SWERVE_DEADBAND ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Constants.Motor.SWERVE_DEADBAND ? ySpeed : 0.0;
    rotationSpeed = Math.abs(rotationSpeed) > Constants.Motor.SWERVE_DEADBAND ? rotationSpeed : 0.0;

    //smooth driving
    /*xSpeed = xLimiter.calculate(xSpeed) * Constants.Motor.SWERVE_MAX_SPEED;
    ySpeed = yLimiter.calculate(ySpeed) * Constants.Motor.SWERVE_MAX_SPEED;
    rotationSpeed = rotationLimiter.calculate(rotationSpeed) * Constants.Motor.SWERVE_MAX_SPEED;*/

    //construct chassis
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xSpeed, ySpeed, rotationSpeed, swerve.getRotation2d());

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
