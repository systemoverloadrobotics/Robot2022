package frc.robot.commands;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class ClimbCommand extends CommandBase {
  Climb climb;

  public ClimbCommand(Climb climb) {
    this.climb = climb;
    addRequirements(climb);
  }

  public Command extendLowClimb() {
    return new Command() {
      @Override
      public void execute() {
        if (climb.getEncoderValue() != Constants.CLIMBER_ENCODER_DISTANCE_LOW) {
          climb.setSetpoint(Constants.CLIMBER_ENCODER_DISTANCE_LOW);
        } else {
          climb.stop();
        }
      }

      @Override
      public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
      }
    }; 
  
  }

  public Command extendMidClimb() {
    return new Command() {
      @Override
      public void execute() {
        if (climb.getEncoderValue() != Constants.CLIMBER_ENCODER_DISTANCE_MID) {
          climb.setSetpoint(Constants.CLIMBER_ENCODER_DISTANCE_MID);
        } else {
          climb.stop();
        }
      }

      @Override
      public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
      }
    };

  }

  public Command retract() {
    return new Command() {
      @Override
      public void execute() {
        climb.setSetpoint(Constants.RETRACTER_ENCODER_DISTANCE);
      }

      @Override
      public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
      }
    }; 
  }

  public void overrideClimb() {
    // todo ovverride climb retract
  }

  public void climbExtend() {
    // todo ovverride climb extend
  }



}
