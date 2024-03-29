package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
//mport frc.robot.Constants.IntakeConstants;
//import frc.robot.subsystems.IntakeSubsystem;

public class ManualClimberCmd extends Command {
  //private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final ClimberSubsystem climberSubsystem;

  public ManualClimberCmd(PivotSubsystem pivotSubsystem, ClimberSubsystem climberSubsystem) {
    //this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.climberSubsystem = climberSubsystem;

    addRequirements(pivotSubsystem, climberSubsystem);
  }

  @Override
  public void initialize() {
    
    pivotSubsystem.setPivotAngle(PivotConstants.INTAKE_ANGLE);
    climberSubsystem.setClimberSpeed(ClimberConstants.CLIMBER_CLIMB_SPEED);
    //intakeSubsystem.setIntakeSpeed(0);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    //shooterSubsystem.setShooterSpeed(0);
  }

  @Override
  public boolean isFinished() {
    if(pivotSubsystem.getIntakeReadiness()){
        climberSubsystem.setClimberSpeed(0);
        return true;
    }
    return false;
  }
}
