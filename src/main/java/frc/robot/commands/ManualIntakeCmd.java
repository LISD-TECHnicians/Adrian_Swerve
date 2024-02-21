package frc.robot.commands;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
//import frc.robot.Constants.IntakeConstants;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ManualIntakeCmd extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;

  public ManualIntakeCmd(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;

    addRequirements(intakeSubsystem, pivotSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pivotSubsystem.setPivotAngle(PivotConstants.INTAKE_ANGLE);

    if (pivotSubsystem.getIntakeReadiness()) {
      intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_DEFAULT_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}