// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.NavigateToTag;
import frc.robot.commands.OrbitAroundReef;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer instance;

  // The robot's subsystems
  public DriveSubsystem m_robotDrive;
  public Navigation m_nav;

  private SendableChooser<Command> m_autonomousChooser = new SendableChooser<Command>();
  private SendableChooser<Pose2d> m_startPosChooser = new SendableChooser<Pose2d>();
  private Command m_autonCommand;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_copilotController = new XboxController(OIConstants.kCopilotControllerPort);
  GenericHID m_buttonBoard = new GenericHID(OIConstants.kButtonBoardPort);
  // ReefController m_reefController = new
  // ReefController(OIConstants.kReefControllerPort);

  public int targetTagId = 0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    instance = this;

    CameraServer.startAutomaticCapture();

    // The robot's subsystems
    m_robotDrive = new DriveSubsystem();
    SmartDashboard.putData("DriveSubsystem", m_robotDrive);
    SmartDashboard.putData("Yaw PID", m_robotDrive.headingPidController);
    m_nav = new Navigation(m_robotDrive);
    SmartDashboard.putData("Navigation", m_nav);
  }

  public void setDefaultCommands() {
    // Configure default commands
    Command teleopCommand = new TeleopDrive(m_robotDrive, m_driverController);
    m_robotDrive.setDefaultCommand(teleopCommand);
    SmartDashboard.putData("TeleopCommand", teleopCommand);

  }

  public static RobotContainer getInstance() {
    return instance;
  }

  public void configureButtonBindings() {

    /**
     * Driver's Controller
     */
    new Trigger(() -> m_driverController.getPOV() == 0)
        .onTrue(new RunCommand(() -> {
          targetTagId = (int) LimelightHelpers.getFiducialID("limelight");
        }));
    new Trigger(() -> m_driverController.getPOV() == 180)
        .whileTrue(Commands.defer(() -> new NavigateToTag(m_robotDrive, m_nav, () -> targetTagId),
            Set.of(m_robotDrive, m_nav)));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new OrbitAroundReef(m_robotDrive, m_nav, 1.0));
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new OrbitAroundReef(m_robotDrive, m_nav, -1.0));
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> {
          double stream = LimelightHelpers.getLimelightNTDouble("limelight", "stream");
          LimelightHelpers.setLimelightNTDouble("limelight", "stream",
              (stream == 0.0 ? 2.0 : 0.0));
        }));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(() -> {
          double newheading = m_robotDrive.getGyroAngleDegrees() + 180.0;
          if (newheading > 180.0) {
            newheading -= 180.0;
          }
          m_robotDrive.setGyroAngleDeg((newheading));
        }));

   

  };

  public void configureTestControls() {
    JoystickButton testPlus = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch1Up);
    JoystickButton testMinus = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch1Down);


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println("getAutonomousCommand()");
    return m_autonCommand;
  }

  public void setAutonCommand(Command cmd) {
    m_autonCommand = cmd;
    System.out.println("setAutonCommand" + m_autonCommand);
  }

  public Pose2d getStartPosition() {
    return m_startPosChooser.getSelected();
  }


  /*
   * called once when is set to Red by the DriverStation
   */
  public void initRed() {
    m_robotDrive.setGyroAngleDeg(0.0);
    m_autonomousChooser = AutonMenus.getRed();
    SmartDashboard.putData("Auton Command", m_autonomousChooser);
    m_autonomousChooser.onChange(this::setAutonCommand);

    m_startPosChooser = StartPositions.getRed();
    SmartDashboard.putData("Start Position", m_startPosChooser);
    m_startPosChooser.onChange(this::setStartPosition);
  }

  /*
   * called once when is set to Blue by the DriverStation
   */
  public void initBlue() {
    m_robotDrive.setGyroAngleDeg(180.0);
    m_autonomousChooser = AutonMenus.getBlue();
    SmartDashboard.putData("Auton Command", m_autonomousChooser);
    m_autonomousChooser.onChange((this::setAutonCommand));

    m_startPosChooser = StartPositions.getBlue();
    SmartDashboard.putData("Start Position", m_startPosChooser);
    m_startPosChooser.onChange(this::setStartPosition);
  }

  private void setStartPosition(Pose2d pose) {
    if (DriverStation.isDisabled()) {
      System.out.println("setStartPosition()" + pose.toString());
      m_robotDrive.setGyroAngleDeg(pose.getRotation().getDegrees());
      m_nav.resetOdometry(pose);
    }
  }

}
