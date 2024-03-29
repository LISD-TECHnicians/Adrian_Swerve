package frc.robot.commands;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
//import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualIntakeCmd extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public ManualIntakeCmd(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(intakeSubsystem, pivotSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    pivotSubsystem.setPivotAngle(PivotConstants.INTAKE_ANGLE);
    shooterSubsystem.setShooterSpeed(0);
    intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_DEFAULT_SPEED);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    //intakeSubsystem.setIntakeSpeed(0);
    //shooterSubsystem.setShooterSpeed(0);
  }

  @Override
  public boolean isFinished() {
    
    if (intakeSubsystem.getNotePresent()) {
      intakeSubsystem.setHoldSpeed(IntakeConstants.INTAKE_HOLD_SPEED);
      shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_HOLD_SPEED);
      pivotSubsystem.setPivotAngle(PivotConstants.TRAVEL_ANGLE);
      return true;
    }
    return false;
  }
}