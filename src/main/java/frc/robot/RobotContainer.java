package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import frc.robot.commands.SwerveCmd;
import frc.robot.commands.ToggleSolenoidCmd;
import frc.robot.commands.SetDriveBrakeCmd;
import frc.robot.commands.SetDriveCoastCmd;
import frc.robot.commands.SetPoseCmd;

import frc.robot.commandgroups.SolenoidPoseCmdGrp;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(ControllerConstants.CONTROLLER_PORT);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final PneumaticsSubsystem pneumaticSubsystem = new PneumaticsSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  private final ToggleSolenoidCmd toggleSolenoid = new ToggleSolenoidCmd(pneumaticSubsystem);
  private final SwerveCmd joystickSwerve = new SwerveCmd(
    swerveSubsystem, 
    () -> -controller.getLeftY(), 
    () -> -controller.getLeftX(), 
    () -> -controller.getRightX(),
    controller.leftTrigger(),
    controller.rightTrigger());
  private final SetPoseCmd resetPose = new SetPoseCmd(swerveSubsystem, DriveConstants.ZERO_POSE);
  private final SetDriveBrakeCmd setDriveBrake = new SetDriveBrakeCmd(swerveSubsystem);
  private final SetDriveCoastCmd setDriveCoast = new SetDriveCoastCmd(swerveSubsystem);

  private final SolenoidPoseCmdGrp solenoidPose = new SolenoidPoseCmdGrp(swerveSubsystem, pneumaticSubsystem);

  public static ShuffleboardTab robotStatus = Shuffleboard.getTab("Robot");
  
  private SendableChooser<Command> autoChooser;// = new SendableChooser<>();

  public RobotContainer() {
    NamedCommands.registerCommand("TEST", new ToggleSolenoidCmd(pneumaticSubsystem));

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

    controller.rightBumper().onTrue(resetPose);

    controller.button(1).onTrue(setDriveBrake);
    controller.button(2).onTrue(setDriveCoast);

    controller.button(3).onTrue(solenoidPose);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}