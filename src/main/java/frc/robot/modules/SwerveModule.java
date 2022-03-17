package frc.robot.modules;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public SwerveModule(TalonFX powerController, TalonSRX steerController, int offSetTicks) {
        this.powerController = powerController;
        this.steerController = steerController;

        powerController.configFactoryDefault();
        steerController.configFactoryDefault();

        powerController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        steerController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

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

        powerController.configPeakOutputForward(0.8);
        powerController.configPeakOutputReverse(-0.8);
        steerController.configPeakOutputForward(0.6);
        steerController.configPeakOutputReverse(-0.6);

        steerController.configNominalOutputForward(Constants.Motor.SWERVE_NOMINAL_OUTPUT_STEER);
        steerController.configNominalOutputReverse(-Constants.Motor.SWERVE_NOMINAL_OUTPUT_STEER);
       
        //steerController.setSelectedSensorPosition(steerController.getSensorCollection().getPulseWidthRiseToFallUs() - offSetTicks);
        steerController.getSensorCollection().syncQuadratureWithPulseWidth(0, 0, true, -offSetTicks + 4096, 50);
    }

    public double getSteerPosition() {
        return (Utils.ticksToDegrees(steerController.getSelectedSensorPosition(), 4096));
    }

    public double getVelocity() {
        return powerController.getSelectedSensorVelocity();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(Utils.sensorUnitsPer100msToMetersPerSecond(getVelocity()),
                Rotation2d.fromDegrees(getSteerPosition()));
    }

    public void setState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        // SmartDashboard.putNumber(steerController.getDeviceID() + "-optimized angle",
        // state.angle.getDegrees());
        powerController.set(ControlMode.Velocity, state.speedMetersPerSecond * Constants.Characteristics.MPS_TO_RPM);
        steerController.set(ControlMode.Position, Utils.degreesToTicks(state.angle.getDegrees(), 4096));
        SmartDashboard.putNumber(steerController.getDeviceID() + "-desired angel", state.angle.getDegrees());
        
    }

    public void stop() {
        powerController.set(ControlMode.PercentOutput, 0);
    }

    public void periodic() {
        // Called at 50hz.
        SmartDashboard.putNumber(steerController.getDeviceID() + "-Steer Postition", steerController.getSelectedSensorPosition());
    }

}
