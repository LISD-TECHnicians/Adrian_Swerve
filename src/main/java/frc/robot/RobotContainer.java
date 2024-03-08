package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.ShooterConstants;
//import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.PivotSubsystem;
//import frc.robot.subsystems.LimelightSubsystem;

import frc.robot.commands.SwerveCmd;
import frc.robot.commands.ManualSubwooferCmd;
import frc.robot.commands.ManualAmpCmd;
//import frc.robot.commands.ManualUpCmd;
import frc.robot.commands.ManualOutCmd;
import frc.robot.commands.ManualScoreCmd;
import frc.robot.commands.ManualIntakeCmd;
import frc.robot.commands.ManualClimberCmd;
import frc.robot.commands.ManualClimbAngleCmd;
import frc.robot.commands.AutoIntakeCmd;
import frc.robot.commands.ManualTravelCmd;
import frc.robot.commands.ManualStopCmd;
//import frc.robot.commands.ToggleSolenoidCmd;
import frc.robot.commands.SetDriveBrakeCmd;
//import frc.robot.commands.SetDriveCoastCmd;
import frc.robot.commands.SetPoseCmd;

//import frc.robot.commandgroups.SolenoidPoseCmdGrp;

import com.pathplanner.lib.auto.AutoBuilder;
//mport com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(ControllerConstants.CONTROLLER_PORT);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  //private final PneumaticsSubsystem pneumaticSubsystem = new PneumaticsSubsystem();
  //private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  //private final ToggleSolenoidCmd toggleSolenoid = new ToggleSolenoidCmd(pneumaticSubsystem);
  private final SwerveCmd joystickSwerve = new SwerveCmd(
    swerveSubsystem, 
    () -> controller.getLeftX(), 
    () -> -controller.getLeftY(), 
    () -> controller.getRightX());//*swerveSubsystem.curvedSpeedOutput(controller.getRightX(), 0),
    //controller.leftTrigger(),
    //controller.rightTrigger());
  private final SetPoseCmd resetPose = new SetPoseCmd(swerveSubsystem, DriveConstants.ZERO_POSE);
  private final SetDriveBrakeCmd setDriveBrake = new SetDriveBrakeCmd(swerveSubsystem);
  //private final SetDriveCoastCmd setDriveCoast = new SetDriveCoastCmd(swerveSubsystem);
  private final ManualSubwooferCmd subwoofer = new ManualSubwooferCmd(intakeSubsystem, pivotSubsystem, shooterSubsystem);
  private final ManualAmpCmd amp = new ManualAmpCmd(intakeSubsystem, pivotSubsystem, shooterSubsystem);
  private final ManualScoreCmd score = new ManualScoreCmd(intakeSubsystem, pivotSubsystem);
  private final ManualIntakeCmd intake = new ManualIntakeCmd(intakeSubsystem, pivotSubsystem, shooterSubsystem);
  private final AutoIntakeCmd autoIntake = new AutoIntakeCmd(intakeSubsystem, pivotSubsystem, shooterSubsystem);
  private final ManualTravelCmd travel = new ManualTravelCmd(intakeSubsystem, pivotSubsystem, shooterSubsystem);
  private final ManualStopCmd stop = new ManualStopCmd(intakeSubsystem, climberSubsystem, shooterSubsystem);
  private final ManualClimberCmd climb = new ManualClimberCmd(pivotSubsystem, climberSubsystem);
  private final ManualClimbAngleCmd climbAngle = new ManualClimbAngleCmd(pivotSubsystem);//, climberSubsystem);
  //private final ManualUpCmd up = new ManualUpCmd(climberSubsystem);
  private final ManualOutCmd out = new ManualOutCmd(intakeSubsystem);

  //private final SolenoidPoseCmdGrp solenoidPose = new SolenoidPoseCmdGrp(swerveSubsystem, pneumaticSubsystem);

  public static ShuffleboardTab robotStatus = Shuffleboard.getTab("Robot");
  
  private SendableChooser<Command> autoChooser;// = new SendableChooser<>();

  public RobotContainer() {
    //NamedCommands.registerCommand("TEST", new ToggleSolenoidCmd(pneumaticSubsystem));
    NamedCommands.registerCommand("SubwooferAim", subwoofer);
    NamedCommands.registerCommand("AmpAim", amp);
    NamedCommands.registerCommand("Score", score);
    NamedCommands.registerCommand("Intake", autoIntake);
    NamedCommands.registerCommand("Travel", travel);
    NamedCommands.registerCommand("ResetPose", resetPose);
    NamedCommands.registerCommand("StopMotors", stop); 

    configureBindings();

    swerveSubsystem.setDefaultCommand(joystickSwerve);

    // autoChooser.setDefaultOption("Toggle Solenoid", toggleSolenoid);
    // autoChooser.addOption("Reset Pose", resetPose);

    autoChooser = AutoBuilder.buildAutoChooser();

    //robotStatus.add("Auto Chooser", autoChooser);
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureBindings() {
    // controller.leftBumper().onTrue(toggleSolenoid);

    controller.button(7).onTrue(resetPose);
    controller.button(8).onTrue(climb);
    //controller.button(9).onTrue(up);
    controller.button(10).onTrue(out);

    //controller.button(5).onTrue(setDriveBrake);
    controller.button(9).onTrue(climbAngle);
    controller.button(4).onTrue(amp);
    controller.button(3).onTrue(subwoofer);
    controller.button(1).onTrue(score);
    controller.button(2).onTrue(intake);
    //controller.button(5).onTrue(travel);
    controller.button(6).onTrue(stop);

    //controller.button(3).onTrue(solenoidPose);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
