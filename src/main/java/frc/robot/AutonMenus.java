// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DoNothing;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.TurnToAprilTag;
import frc.robot.subsystems.Navigation;

/** Add your docs here. */
public class AutonMenus {

    public static SendableChooser<Command> getBlue(){
        RobotContainer bot = RobotContainer.getInstance();
    
        SendableChooser<Command> blueCommands = new SendableChooser<Command>();
        blueCommands.setDefaultOption("DoNothing (blue)", new DoNothing());
        blueCommands.addOption("Drive to april tag 1", new DriveToAprilTag(bot.m_robotDrive, bot.m_nav, 1));
        blueCommands.addOption("go fwd 1m", GoToCommand.relative(bot.m_robotDrive, bot.m_nav, 1.0, 0, 0));
        blueCommands.addOption("go fwd 2m", GoToCommand.relative(bot.m_robotDrive, bot.m_nav, 2.0, 0, 0));
        blueCommands.addOption("GoTo -2, 0, 0", GoToCommand.relative(bot.m_robotDrive, bot.m_nav, -2.0, 0, 0));
        blueCommands.addOption("GoTo 1, -1, 0", GoToCommand.relative(bot.m_robotDrive, bot.m_nav, 1.0, -1.0, 0));
        blueCommands.addOption("Nav to tag 1", GoToCommand.absolute(bot.m_robotDrive, bot.m_nav, Navigation.getPose2dInFrontOfTag(1, 0.5)));
        blueCommands.addOption("Nav to tag 17", GoToCommand.absolute(bot.m_robotDrive, bot.m_nav, Navigation.getPose2dInFrontOfTag(17, 0.5)));
        blueCommands.addOption("Nav to tag 18", GoToCommand.absolute(bot.m_robotDrive, bot.m_nav, Navigation.getPose2dInFrontOfTag(18, 0.5)));
        blueCommands.addOption("Nav to tag 19", GoToCommand.absolute(bot.m_robotDrive, bot.m_nav, Navigation.getPose2dInFrontOfTag(19, 0.5)));
        
        return blueCommands;
    }

    public static SendableChooser<Command> getRed(){
        RobotContainer bot = RobotContainer.getInstance();

        SendableChooser<Command> redCommands = new SendableChooser<Command>();
        redCommands.setDefaultOption("Do Nothing (red)", new DoNothing());
        redCommands.addOption("turn to april tag B 10", new TurnToAprilTag(bot.m_robotDrive, 10));
        redCommands.addOption("turn to april tag 1", new TurnToAprilTag(bot.m_robotDrive, 1));
        redCommands.addOption("turn to april tag 11", new TurnToAprilTag(bot.m_robotDrive, 11));
        redCommands.addOption("Drive to april tag 1", new DriveToAprilTag(bot.m_robotDrive, bot.m_nav, 1));
        redCommands.addOption("GoTo 1, 0, 0", GoToCommand.relative(bot.m_robotDrive, bot.m_nav, 1.0, 0, 0));
        redCommands.addOption("GoTo 2, 0, 0", GoToCommand.relative(bot.m_robotDrive, bot.m_nav, 2.0, 0, 0));
        redCommands.addOption("GoTo -2, 0, 0", GoToCommand.relative(bot.m_robotDrive, bot.m_nav, -2.0, 0, 0));
        redCommands.addOption("GoTo 1, -1, 0", GoToCommand.relative(bot.m_robotDrive, bot.m_nav, 1.0, -1.0, 0));
        redCommands.addOption("Nav to tag 1", GoToCommand.absolute(bot.m_robotDrive, bot.m_nav, Navigation.getPose2dInFrontOfTag(1, 0.5)));
        redCommands.addOption("Nav to tag 17", GoToCommand.absolute(bot.m_robotDrive, bot.m_nav, Navigation.getPose2dInFrontOfTag(17, 0.5)));
        redCommands.addOption("Nav to tag 18", GoToCommand.absolute(bot.m_robotDrive, bot.m_nav, Navigation.getPose2dInFrontOfTag(18, 0.5)));
        redCommands.addOption("Nav to tag 19", GoToCommand.absolute(bot.m_robotDrive, bot.m_nav, Navigation.getPose2dInFrontOfTag(19, 0.5)));
        
        return redCommands;
    }
}
