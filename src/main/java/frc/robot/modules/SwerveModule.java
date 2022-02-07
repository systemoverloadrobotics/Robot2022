package frc.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.util.Utils;

// SwerveModule manages an individual swerve drive module on the robot.
public class SwerveModule {

    // Should contain the two motor controllers, tracking, methods for setting,
    // and encoder output.

    private TalonFX powerController;
    private TalonSRX steerController;

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

    public SwerveModule(TalonFX powerController, TalonSRX steerController, double offSetDeg) {
        this.powerController = powerController;
        this.steerController = steerController;
        this.offSetDeg = offSetDeg;

        powerController.configFactoryDefault(); 
        steerController.configFactoryDefault();

        powerController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        steerController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        powerController.config_kP(0, Constants.PID.P_SWERVE_POWER);
        powerController.config_kI(0, Constants.PID.I_SWERVE_POWER);
        powerController.config_kD(0, Constants.PID.D_SWERVE_POWER);

        steerController.config_kP(0, Constants.PID.P_SWERVE_STEER);
        steerController.config_kI(0, Constants.PID.D_SWERVE_STEER);
        steerController.config_kD(0, Constants.PID.I_SWERVE_STEER);

        powerController.configNominalOutputForward(Constants.Motor.SWERVE_NOMINAL_OUTPUT_PERCENT);
        powerController.configNominalOutputReverse(Constants.Motor.SWERVE_NOMINAL_OUTPUT_PERCENT);

        steerController.configNominalOutputForward(Constants.Motor.SWERVE_NOMINAL_OUTPUT_PERCENT);
        steerController.configNominalOutputReverse(Constants.Motor.SWERVE_NOMINAL_OUTPUT_PERCENT);

        powerSim = powerController.getSimCollection();
        steerSim = steerController.getSimCollection();
    }

    public void setSteerRotation(double angle){
        steerController.set(ControlMode.Position, Utils.degreesToTicks(angle, 4096));
    }

    public double getSteerPosition(){
        return Utils.ticksToDegrees(steerController.getSelectedSensorPosition(), 4096);
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
        return new SwerveModuleState( getVelocity(), new Rotation2d(getSteerPosition()));
    }

    public void setState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        powerController.set(ControlMode.Velocity, state.speedMetersPerSecond);
        steerController.set(ControlMode.Position, Utils.degreesToTicks(state.angle.getDegrees(), 4096));
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
