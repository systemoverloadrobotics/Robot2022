package frc.robot.modules;

import java.nio.charset.StandardCharsets;
import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Utils;

// SwerveModule manages an individual swerve drive module on the robot.
public class SwerveModule {

    // Should contain the two motor controllers, tracking, methods for setting,
    // and encoder output.

    private TalonFX powerController;
    private TalonSRX steerController;

    private PIDController powePIDController;
    private PIDController steerPIDController;

    // Linear drive feed forward
    public SimpleMotorFeedforward driveFF = Constants.PID.DRIVE_FF;
    // Steer feed forward
    public SimpleMotorFeedforward steerFF = Constants.PID.STEER_FF;

    //Simulation
    private TalonFXSimCollection powerSim;
    private TalonSRXSimCollection steerSim;

    private double offSetDeg;

    private final FlywheelSim powerMotorSimModel = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            driveFF.kv * Constants.RobotDimensions.WHEEL_CIRCUMFRENCE / (2*Math.PI),
            driveFF.ka * Constants.RobotDimensions.WHEEL_CIRCUMFRENCE / (2*Math.PI)
        ),
        DCMotor.getFalcon500(1),
        Constants.Motor.SWERVE_POWER_GEAR_RATIO
    );

    private final FlywheelSim steerMotorSimModel = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(steerFF.kv,steerFF.ka),
        DCMotor.getFalcon500(1),
        Constants.Motor.SWERVE_POWER_GEAR_RATIO
    );

    public SwerveModule(TalonFX powerController, TalonSRX steerController) {
        this.powerController = powerController;
        this.steerController = steerController;

        powerController.configFactoryDefault(); 
        steerController.configFactoryDefault();

        powerController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        steerController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        powerController.config_kF(0, 1000);
        powerController.config_kP(0, Constants.PID.P_SWERVE_POWER);
        powerController.config_kI(0, Constants.PID.I_SWERVE_POWER);
        powerController.config_kD(0, Constants.PID.D_SWERVE_POWER);

        steerController.config_kF(0, 0);
        // steerController.config_kP(0, Constants.PID.P_SWERVE_STEER);
        // steerController.config_kI(0, Constants.PID.I_SWERVE_STEER);
        // steerController.config_kD(0, Constants.PID.D_SWERVE_STEER);
        steerPIDController = new PIDController(Constants.PID.P_SWERVE_STEER, Constants.PID.I_SWERVE_STEER, Constants.PID.D_SWERVE_STEER);

        powerController.configNominalOutputForward(Constants.Motor.SWERVE_NOMINAL_OUTPUT_PERCENT);
        powerController.configNominalOutputReverse(-Constants.Motor.SWERVE_NOMINAL_OUTPUT_PERCENT);

        powerController.configPeakOutputForward(0.4);
        powerController.configPeakOutputReverse(0.4);
        steerController.configPeakOutputForward(0.4);
        steerController.configPeakOutputReverse(0.4);

        steerController.configNominalOutputForward(Constants.Motor.SWERVE_NOMINAL_OUTPUT_STEER);
        steerController.configNominalOutputReverse(-Constants.Motor.SWERVE_NOMINAL_OUTPUT_STEER);

        // steerController.configFeedbackNotContinuous(false, 50);

        powerSim = powerController.getSimCollection();
        steerSim = steerController.getSimCollection();
    }

    public void setSteerRotation(double angle){
        steerController.set(ControlMode.Position, Utils.degreesToTicks(angle, 4096));
    }

    public double getSteerPosition(){
        return Utils.ticksToDegrees(steerController.getSelectedSensorPosition(), 4096) % 360;
    }
    
    public void resetEncoder(){
        double angle = getSteerPosition() - offSetDeg;
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
        return new SwerveModuleState(getVelocity(), new Rotation2d(Math.toRadians(getSteerPosition())));
    }

    public void setState(SwerveModuleState state){
        SmartDashboard.putNumber(powerController.getDeviceID() + "-power in RPM", state.speedMetersPerSecond * Constants.Characteristics.MPS_TO_RPM);
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        // state = SwerveModuleState.optimize(state, getState().angle);
        powerController.set(ControlMode.PercentOutput, state.speedMetersPerSecond / Constants.Motor.SWERVE_MAX_SPEED);
        SmartDashboard.putNumber(steerController.getDeviceID() + " Steer Position", Math.abs(getSteerPosition() % 360));
        SmartDashboard.putNumber(steerController.getDeviceID() + " setpoint", Math.abs(state.angle.getDegrees()));
        if(Math.abs((int) getSteerPosition() % 360) != Math.abs((int) state.angle.getDegrees())){
            steerController.set(ControlMode.PercentOutput, 0.1);
        }else{
            steerController.set(ControlMode.PercentOutput, 0);
        }
        SmartDashboard.putNumber(steerController.getDeviceID() + "-steer setpoint", Utils.degreesToTicks(state.angle.getDegrees(), 4096));
    }

    public void stop(){
        powerController.set(ControlMode.PercentOutput, 0);
        steerController.set(ControlMode.PercentOutput, 0);
    }
  
    public void periodic() {
        // Called at 50hz.
    }

    public void simulationPeriodic(){
        powerMotorSimModel.setInput(powerController.getMotorOutputVoltage());
        steerMotorSimModel.setInput(steerController.getMotorOutputVoltage());
        powerMotorSimModel.update(0.02);
        steerMotorSimModel.update(0.2);  
    }
}
