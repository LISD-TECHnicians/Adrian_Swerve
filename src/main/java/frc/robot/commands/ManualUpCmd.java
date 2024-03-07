package frc.robot.commands;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.IntakeConstants;
//import frc.robot.Constants.PivotConstants;
//import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
//import frc.robot.subsystems.PivotSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;

public class ManualUpCmd extends Command {
  private final ClimberSubsystem climberSubsystem;
  //private final PivotSubsystem pivotSubsystem;
  //private final ShooterSubsystem shooterSubsystem;

  public ManualUpCmd(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    //this.pivotSubsystem = pivotSubsystem;
    //this.shooterSubsystem = shooterSubsystem;

    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {
    climberSubsystem.setClimberSpeed(ClimberConstants.CLIMBER_CLIMB_SPEED);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    //climberSubsystem.setClimberSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
