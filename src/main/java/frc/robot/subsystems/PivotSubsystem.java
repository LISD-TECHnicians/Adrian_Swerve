package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PivotConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotSubsystem extends SubsystemBase {
  private final CANSparkMax pivot = new CANSparkMax(PivotConstants.PIVOT_ID, MotorType.kBrushless);

  private final DutyCycleEncoder pivot_encoder = new DutyCycleEncoder(PivotConstants.PIVOT_ENCODER_ID);

  private final PIDController pivotPID = new PIDController(PivotConstants.PIVOT_P, PivotConstants.PIVOT_I, PivotConstants.PIVOT_D);

  // Encoder in Right SparkMax, Limits in Left SparkMax

  public PivotSubsystem() {
    pivot.restoreFactoryDefaults();

    pivot.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);

    pivot.setSmartCurrentLimit(PivotConstants.PIVOT_CURRENT_LIMIT);

    pivot.setIdleMode(IdleMode.kBrake);

    //pivot.setSoftLimit(SoftLimitDirection.kForward, PivotConstants.PIVOT_FORWARD_LIMIT);
    //pivot.setSoftLimit(SoftLimitDirection.kReverse, PivotConstants.PIVOT_REVERSE_LIMIT);

    //pivot.enableSoftLimit(SoftLimitDirection.kForward, true);
    //pivot.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  public void setPivotAngle(double angle) {
    pivot.set(MathUtil.clamp(pivotPID.calculate(getPivotAngle(), angle), -PivotConstants.PIVOT_MAX_OUTPUT, PivotConstants.PIVOT_MAX_OUTPUT));
  }

  public double getPivotRawAngle(){
    return pivot_encoder.getAbsolutePosition();
  }

  public double getPivotAngle() {
    return ((PivotConstants.ANGLE_OFFSET-pivot_encoder.getAbsolutePosition())*360); //  degrees
  }

  public double getFeedForward(){
    return Math.cos(getPivotAngle()-PivotConstants.INTAKE_ANGLE)*PivotConstants.PIVOT_MIN_OUTPUT;
  }

  public boolean getShooterReadiness(double angle) {
    return Math.abs(getPivotAngle() - angle) < PivotConstants.PIVOT_VARIABILITY;
  }

  public boolean getSubwooferReadiness() {
    return Math.abs(getPivotAngle() - PivotConstants.SUBWOOFER_ANGLE) < PivotConstants.PIVOT_VARIABILITY;
  }

  public boolean getIntakeReadiness() {
    return Math.abs(getPivotAngle() - PivotConstants.INTAKE_ANGLE) < PivotConstants.PIVOT_VARIABILITY;
  }

  public boolean getTravelReadiness() {
    return Math.abs(getPivotAngle() - PivotConstants.TRAVEL_ANGLE) < PivotConstants.PIVOT_VARIABILITY;
  }

  public boolean getAmpReadiness() {
    return Math.abs(getPivotAngle() - PivotConstants.AMP_ANGLE) < PivotConstants.PIVOT_VARIABILITY;
  }

  @Override
  public void periodic() {
    System.out.println("Raw Encoder Position: " + getPivotAngle());
  }

  @Override
  public void simulationPeriodic() {}
}