package frc.robot.commands.auto;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.StorageCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Storage.ToggleState;

public class AutoPaths {
    /* Sample Auto for Swerve */
    private static Swerve swerve;
    private static Intake intake;
    private static Storage storage;
    private static Shooter shooter;
    private static Limelight limelight;



    public AutoPaths(Swerve swerve, Intake intake, Storage storage, Shooter shooter,
            Limelight limelight) {
        this.swerve = swerve;
        this.intake = intake;
        this.storage = storage;
        this.shooter = shooter;
        this.limelight = limelight;
    }

    private static SwerveControllerCommand moveDriveTrain(int startPoint, int endPoint,
            int rotation) {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.Motor.SWERVE_MAX_SPEED,
                Constants.Motor.SWERVE_MAX_ACCELERATION);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(0, startPoint), new Translation2d(0, endPoint)),
                new Pose2d(0, endPoint, Rotation2d.fromDegrees(rotation)), trajectoryConfig);

        PIDController xController = new PIDController(Constants.PID.P_X_CONTROLLER, 0, 0);
        PIDController yController = new PIDController(Constants.PID.P_Y_CONTROLLER, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.PID.P_THETA_CONTROLLER, 0, 0, Constants.Motor.THETA_CONTROL_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new SwerveControllerCommand(trajectory, swerve::getPose,
                Constants.Motor.SWERVE_DRIVE_KINEMATICS, xController, yController, thetaController,
                swerve::setModuleStates, swerve);
    }


    public static Command twoBallAuto() {
        SwerveControllerCommand initialMovementCommand = moveDriveTrain(1, 7, 0);

        SwerveControllerCommand moveToShoot = moveDriveTrain(1, 9, 0);


        IntakeBall intakeCommand = new IntakeBall(intake);
        ShooterCommand shooterCommand = new ShooterCommand(limelight, shooter);
        StorageCommand storageCommand = new StorageCommand(storage);

        return new SequentialCommandGroup(
                new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
                new ParallelCommandGroup(initialMovementCommand, intakeCommand, storageCommand),
                new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
                moveToShoot, shooterCommand);
    }

    public static Command exampleAuto() {
        // create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.Motor.SWERVE_MAX_SPEED,
                Constants.Motor.SWERVE_MAX_ACCELERATION)
                        .setKinematics(Constants.Motor.SWERVE_DRIVE_KINEMATICS);

        // generate trajectory
        Trajectory trajectory =
                TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(new Translation2d(1, 0), new Translation2d(1, -1)),
                        new Pose2d(2, -1, Rotation2d.fromDegrees(180)), trajectoryConfig);

        // pid controllers for tracking trajectory
        PIDController xController = new PIDController(Constants.PID.P_X_CONTROLLER, 0, 0);
        PIDController yController = new PIDController(Constants.PID.P_Y_CONTROLLER, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.PID.P_THETA_CONTROLLER, 0, 0, Constants.Motor.THETA_CONTROL_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
                swerve::getPose, Constants.Motor.SWERVE_DRIVE_KINEMATICS, xController, yController,
                thetaController, swerve::setModuleStates, swerve);

        return new SequentialCommandGroup(
                new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand, new InstantCommand(() -> swerve.stopModules()));


    }

}
