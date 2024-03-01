package frc.robot.commands;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
//import frc.robot.Constants.IntakeConstants;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualTravelCmd extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public ManualTravelCmd(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(intakeSubsystem, pivotSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pivotSubsystem.setPivotAngle(PivotConstants.TRAVEL_ANGLE);
    shooterSubsystem.setShooterSpeed(-0.1);
    intakeSubsystem.setIntakeSpeed(-0.1);

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}