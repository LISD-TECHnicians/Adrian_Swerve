package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualSubwooferCmd extends Command {
  private final PivotSubsystem pivotSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public ManualSubwooferCmd(PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem) {
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(pivotSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pivotSubsystem.setPivotAngle(PivotConstants.SUBWOOFER_ANGLE);
    shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SHOOT_SPEED);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}