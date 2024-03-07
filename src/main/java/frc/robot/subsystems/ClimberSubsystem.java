package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax climberLeft = new CANSparkMax(ClimberConstants.CLIMBER_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax climberRight = new CANSparkMax(ClimberConstants.CLIMBER_RIGHT_ID, MotorType.kBrushless);

  public ClimberSubsystem() {
    climberLeft.restoreFactoryDefaults();
    climberRight.restoreFactoryDefaults();

    climberLeft.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);
    climberRight.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);

    climberLeft.setSmartCurrentLimit(ControllerConstants.DEFAULT_NEO_CURRENT_LIMIT);
    climberRight.setSmartCurrentLimit(ControllerConstants.DEFAULT_NEO_CURRENT_LIMIT);

    climberLeft.setIdleMode(IdleMode.kBrake);
    climberRight.setIdleMode(IdleMode.kBrake);

    climberRight.follow(climberLeft, true);
  }

  public void setClimberSpeed(double speed) {
    climberLeft.set(speed * ClimberConstants.CLIMBER_SPEED_FACTOR);
  }

  public double getClimberSpeed() {
    return climberLeft.get();
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}