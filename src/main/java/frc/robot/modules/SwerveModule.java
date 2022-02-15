package frc.robot.modules;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Utils;

// SwerveModule manages an individual swerve drive module on the robot.
public class SwerveModule {

    // Should contain the two motor controllers, tracking, methods for setting,
    // and encoder output.

    private TalonFX powerController;
    private TalonSRX steerController;

    private double offSetTicks;
    // Linear drive feed forward
    public SimpleMotorFeedforward driveFF = Constants.PID.DRIVE_FF;
    // Steer feed forward
    public SimpleMotorFeedforward steerFF = Constants.PID.STEER_FF;

    public SwerveModule(TalonFX powerController, TalonSRX steerController, double offSetTicks) {
        this.powerController = powerController;
        this.steerController = steerController;
        this.offSetTicks = offSetTicks;

        powerController.configFactoryDefault(); 
        steerController.configFactoryDefault();

        powerController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        steerController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        steerController.setSensorPhase(true);
        steerController.configFeedbackNotContinuous(false, 50);
        steerController.setInverted(false);
 
        powerController.config_kF(0, 1000);
        powerController.config_kP(0, Constants.PID.P_SWERVE_POWER);
        powerController.config_kI(0, Constants.PID.I_SWERVE_POWER);
        powerController.config_kD(0, Constants.PID.D_SWERVE_POWER);

        steerController.config_kF(0, 0);
        steerController.config_kP(0, Constants.PID.P_SWERVE_STEER);
        steerController.config_kI(0, Constants.PID.I_SWERVE_STEER);
        steerController.config_kD(0, Constants.PID.D_SWERVE_STEER);

        powerController.configNominalOutputForward(Constants.Motor.SWERVE_NOMINAL_OUTPUT_PERCENT);
        powerController.configNominalOutputReverse(-Constants.Motor.SWERVE_NOMINAL_OUTPUT_PERCENT);

        powerController.configPeakOutputForward(0.5);
        powerController.configPeakOutputReverse(-0.5);
        steerController.configPeakOutputForward(0.6);
        steerController.configPeakOutputReverse(-0.6);

        steerController.configNominalOutputForward(Constants.Motor.SWERVE_NOMINAL_OUTPUT_STEER);
        steerController.configNominalOutputReverse(-Constants.Motor.SWERVE_NOMINAL_OUTPUT_STEER);
    }

    public void setSteerRotation(double angle){
        steerController.set(ControlMode.Position, Utils.degreesToTicks(angle, 4096));
    }

    public double getSteerPosition(){
        return -(Utils.ticksToDegrees(steerController.getSelectedSensorPosition(), 4096))
;
    }
    
    public void resetEncoder(){
        double angle = getSteerPosition();
        powerController.set(ControlMode.Position, 0);
        steerController.setSelectedSensorPosition(angle);
        
    }

    public void setVelocity(double percent){
        powerController.set(ControlMode.PercentOutput, percent);
    }

    public double getVelocity(){
        return powerController.getSelectedSensorVelocity();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(Utils.sensorUnitsPer100msToMetersPerSecond(getVelocity()), Rotation2d.fromDegrees(getSteerPosition()));
    }

    public void setState(SwerveModuleState state, double rotationSpeed){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        
        state = optimizeDirection(state);
        //state = SwerveModuleState.optimize(state, getState().angle);
        //SmartDashboard.putNumber(steerController.getDeviceID() + "-optimized angle", state.angle.getDegrees());
        powerController.set(ControlMode.PercentOutput, state.speedMetersPerSecond / Constants.Motor.SWERVE_MAX_SPEED);
        steerController.set(ControlMode.Position, (Utils.degreesToTicks(state.angle.getDegrees(), 4096) + 2048) + offSetTicks + 1024);
    }

    public static Vector2d getWheelVector(SwerveModuleState state, double rotationSpeed, ModuleLocation location) {
        Vector2d rotationVector;
        switch (location) {
            case FL:
                rotationVector = new Vector2d(Constants.RobotDimensions.WIDTH / 2, -Constants.RobotDimensions.LENGTH / 2);
            case FR:
                rotationVector = new Vector2d(-Constants.RobotDimensions.WIDTH / 2, Constants.RobotDimensions.LENGTH / 2);
            case BL:
                rotationVector = new Vector2d(Constants.RobotDimensions.WIDTH / 2, Constants.RobotDimensions.LENGTH / 2);
            case BR:
            default:
                rotationVector = new Vector2d(-Constants.RobotDimensions.WIDTH / 2, -Constants.RobotDimensions.LENGTH / 2);
        }

        // CCW
        rotationVector.rotate(90);
        rotationVector = new Vector2d(rotationVector.x / rotationVector.magnitude(), rotationVector.y / rotationVector.magnitude());
        double scale = rotationSpeed * -1;
        rotationVector = new Vector2d(rotationVector.x * scale, rotationVector.y * scale);
        Vector2d strafingVector = Utils.atan2Inverse(state.angle.getRadians(), state.speedMetersPerSecond);
        Vector2d wheelVector = Utils.addVectors(strafingVector, rotationVector);

        return wheelVector;
    }

    public SwerveModuleState optimizeDirection(SwerveModuleState state) {
        double currentAngle = getSteerPosition();
        double desiredAngle = state.angle.getDegrees();
        double possibleAngleFWD = desiredAngle;
        double possibleAngleREV = (desiredAngle + 180) % 360;
        double angleDifferenceFWD = Math.abs(possibleAngleFWD - currentAngle);
        double angleDifferenceREV = Math.abs(possibleAngleREV - currentAngle);
        
        return new SwerveModuleState(
            angleDifferenceFWD < angleDifferenceREV ? state.speedMetersPerSecond : -state.speedMetersPerSecond, 
            Rotation2d.fromDegrees(Math.min(angleDifferenceFWD, angleDifferenceREV))
        );
    }

    public void stop(){
        powerController.set(ControlMode.PercentOutput, 0);
    }
  
    public void periodic() {
        // Called at 50hz.
    }

    public ModuleLocation getModuleLocation() {
        switch (steerController.getDeviceID()) {
            case Constants.Motor.SWERVE_FRONT_LEFT_STEER:
                return ModuleLocation.FL;
            case Constants.Motor.SWERVE_FRONT_RIGHT_STEER:
                return ModuleLocation.FR;
            case Constants.Motor.SWERVE_BACK_LEFT_STEER:
                return ModuleLocation.BL;
            case Constants.Motor.SWERVE_BACK_RIGHT_STEER:
            default:
                return ModuleLocation.BR;
            
        }
    }

    public enum ModuleLocation {
        FL, FR, BL, BR;
    }

}
