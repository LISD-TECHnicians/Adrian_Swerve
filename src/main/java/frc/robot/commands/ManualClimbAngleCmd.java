package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;
//import frc.robot.Constants.ClimberConstants;
//import frc.robot.subsystems.ClimberSubsystem;
//mport frc.robot.Constants.IntakeConstants;
//import frc.robot.subsystems.IntakeSubsystem;

public class ManualClimbAngleCmd extends Command {
  //private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;
  //private final ClimberSubsystem climberSubsystem;

  public ManualClimbAngleCmd(PivotSubsystem pivotSubsystem) {//, ClimberSubsystem climberSubsystem) {
    //this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    //this.climberSubsystem = climberSubsystem;

    addRequirements(pivotSubsystem);//, climberSubsystem);
  }

  @Override
  public void initialize() {
    
    pivotSubsystem.setPivotAngle(PivotConstants.CLIMB_ANGLE);
    //climberSubsystem.setClimberSpeed(ClimberConstants.CLIMBER_CLIMB_SPEED);
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
    return true;
  }
}
