package frc.robot.commands;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.IntakeConstants;
//import frc.robot.Constants.PivotConstants;
//import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.ClimberSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;

public class ManualOutCmd extends Command {
  private final IntakeSubsystem intakeSubsystem;
  //private final ClimberSubsystem climberSubsystem;
  //private final ShooterSubsystem shooterSubsystem;

  public ManualOutCmd(IntakeSubsystem intakeSubsystem) {//}, ClimberSubsystem climberSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    //this.climberSubsystem = climberSubsystem;
    //this.shooterSubsystem = shooterSubsystem;

    addRequirements(intakeSubsystem);//, climberSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    //shooterSubsystem.setShooterSpeed(0);
    intakeSubsystem.setIntakeSpeed(-1*IntakeConstants.INTAKE_DEFAULT_SPEED);
    //climberSubsystem.setClimberSpeed(0);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}