// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.NavRoute;
import frc.robot.commands.NavigateToTag;
import frc.robot.commands.StopCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TestDrive;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static Robot instance;
  public static DriverStation.Alliance alliance;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  Robot(){}

  Robot(double period) {
    super(period);
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Starts recording to data log
    DataLogManager.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //DataLogManager.start();
    //CanBridge.runTCP();
    LimelightHelpers.setCameraPose_RobotSpace(Constants.LimelightConstants.name,
                                              Constants.LimelightConstants.Offset.forward,
                                              Constants.LimelightConstants.Offset.side,
                                              Constants.LimelightConstants.Offset.up,
                                              Constants.LimelightConstants.Offset.roll,
                                              Constants.LimelightConstants.Offset.pitch,
                                              Constants.LimelightConstants.Offset.yaw
                                            );
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * Dashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }


  @Override
  public void disabledPeriodic() {
    if(alliance == null) {
      Optional<Alliance> a = DriverStation.getAlliance();
      if (a.isPresent()) {
        alliance = a.get();
        if(alliance == Alliance.Red) {
          m_robotContainer.initRed();
        } else {
          m_robotContainer.initBlue();
        }
      }
    }
  }


  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    System.out.println("autonomousInit() m_pos == " + m_robotContainer.m_nav.getPose());
    System.out.println("Starting command: " + m_autonomousCommand.getName());


    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

    m_robotContainer.configureButtonBindings();
    m_robotContainer.setDefaultCommands();
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (30.0 > DriverStation.getMatchTime() && DriverStation.getMatchTime() > 29.0) {
      m_robotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 1.0);
      m_robotContainer.m_copilotController.setRumble(RumbleType.kBothRumble, 1.0);
    } else {
      m_robotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 0.0);
      m_robotContainer.m_copilotController.setRumble(RumbleType.kBothRumble, 0.0);
    }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    m_robotContainer.m_robotDrive.stop();

    m_robotContainer.configureTestControls();

    Command testDriveCommand = new TeleopDrive(m_robotContainer.m_robotDrive, m_robotContainer.m_driverController);
    m_robotContainer.m_robotDrive.setDefaultCommand(new TestDrive(m_robotContainer.m_robotDrive, m_robotContainer.m_driverController));
    SmartDashboard.putData("TestDrive", testDriveCommand);

    SmartDashboard.putData("GoToCommand 0, 0", GoToCommand.absolute(m_robotContainer.m_robotDrive, m_robotContainer.m_nav, 0, 0, 0));
    SmartDashboard.putData("GoToCommand 0, 6", GoToCommand.absolute(m_robotContainer.m_robotDrive, m_robotContainer.m_nav, 0, 6, 0));
    SmartDashboard.putData("GoToCommand 6, 0", GoToCommand.absolute(m_robotContainer.m_robotDrive, m_robotContainer.m_nav, 6, 0, 0));
    SmartDashboard.putData("GoToCommand 6, 6", GoToCommand.absolute(m_robotContainer.m_robotDrive, m_robotContainer.m_nav, 6, 6, 0));
    SmartDashboard.putData("Nav to tag 21", new NavigateToTag(m_robotContainer.m_robotDrive,
                                                                  m_robotContainer.m_nav,
                                                                  ()->{return 21;}
                                                                  ));
    SmartDashboard.putData("Nav to tag  8", new NavigateToTag(m_robotContainer.m_robotDrive,
                                                                  m_robotContainer.m_nav,
                                                                  ()->{return 8;}
                                                                  ));
    SmartDashboard.putData("StopCommand", new StopCommand(m_robotContainer.m_robotDrive));

    SmartDashboard.putData("TestRoute", new NavRoute(m_robotContainer.m_robotDrive,
                                                      m_robotContainer.m_nav,
                                                      List.of(new Pose2d(0, 0, new Rotation2d(0)),
                                                              new Pose2d(6, 0, new Rotation2d(0)),
                                                              new Pose2d(6, 6, new Rotation2d(0)),
                                                              new Pose2d(0, 6, new Rotation2d(0))
                                                              )
                                                      ));
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
    m_robotContainer.m_robotDrive.removeDefaultCommand();
  }


}
