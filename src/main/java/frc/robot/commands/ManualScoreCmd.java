package frc.robot.commands;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
//import frc.robot.Constants.ShooterConstants;
//import frc.robot.Constants.PivotConstants;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;


public class ManualScoreCmd extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;
  //private final ShooterSubsystem shooterSubsystem;

  public ManualScoreCmd(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    //this.shooterSubsystem = shooterSubsystem;

    addRequirements(intakeSubsystem, pivotSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (pivotSubsystem.getAmpReadiness()) {
      intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_DEFAULT_SPEED);
      //shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_AMP_SPEED);
      return true;
    }

    if (pivotSubsystem.getSubwooferReadiness()) {
      intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_DEFAULT_SPEED);
      //shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SHOOT_SPEED);
      return true;
    }
    
    System.out.println("Not At Right Angle");
    return false;
  }
}