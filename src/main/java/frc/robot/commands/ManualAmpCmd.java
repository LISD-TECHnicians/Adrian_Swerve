package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class ManualAmpCmd extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public ManualAmpCmd(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(intakeSubsystem, pivotSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    pivotSubsystem.setPivotAngle(PivotConstants.AMP_ANGLE);
    shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_AMP_SPEED);
    intakeSubsystem.setIntakeSpeed(0);
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
