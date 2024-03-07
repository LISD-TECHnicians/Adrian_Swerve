package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
//mport frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class ManualSubwooferCmd extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public ManualSubwooferCmd(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(intakeSubsystem, pivotSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    
    pivotSubsystem.setPivotAngle(PivotConstants.SUBWOOFER_ANGLE);
    shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SHOOT_SPEED);
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
