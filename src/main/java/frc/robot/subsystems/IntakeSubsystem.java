package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intakeLeft = new CANSparkMax(IntakeConstants.INTAKE_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax intakeRight = new CANSparkMax(IntakeConstants.INTAKE_RIGHT_ID, MotorType.kBrushless);
  private final DigitalInput noteSensor = new DigitalInput(1);

  public IntakeSubsystem() {
    intakeLeft.restoreFactoryDefaults();
    intakeRight.restoreFactoryDefaults();

    intakeLeft.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);
    intakeRight.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);

    intakeLeft.setSmartCurrentLimit(ControllerConstants.DEFAULT_NEO_CURRENT_LIMIT);
    intakeRight.setSmartCurrentLimit(ControllerConstants.DEFAULT_NEO_CURRENT_LIMIT);

    intakeLeft.setIdleMode(IdleMode.kBrake);
    intakeRight.setIdleMode(IdleMode.kBrake);

    //intakeRight.follow(intakeLeft, false);
  }

  public void setIntakeSpeed(double speed) {
    intakeLeft.set(speed * IntakeConstants.INTAKE_SPEED_FACTOR);
    intakeRight.set(speed * IntakeConstants.INTAKE_SPEED_FACTOR);
  }

  public void setHoldSpeed(double speed) {
    intakeLeft.set(speed * IntakeConstants.INTAKE_SPEED_FACTOR);
    intakeRight.set(0);
  }

  public double getIntakeSpeed() {
    return intakeLeft.get();
  }

  public boolean getNotePresent() {
    return !noteSensor.get();
  }

  @Override
  public void periodic() {
    //System.out.println("Note Present: " + getNotePresent());
  }

  @Override
  public void simulationPeriodic() {}
}