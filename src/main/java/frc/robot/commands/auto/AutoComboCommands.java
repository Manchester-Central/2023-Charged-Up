// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.Grip;
import frc.robot.commands.MoveArm;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.Gripper;
import frc.robot.subsystems.swerve.DrivePose;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * A class for combining other commands into single auto commands
 */
public class AutoComboCommands {
  
  public static Command driveAndMoveArm(ParsedCommand parsedCommand, SwerveDrive swerveDrive, Arm arm) {
    var driveCommand = DriveToTarget.createAutoCommand(parsedCommand, swerveDrive);
    var armCommand = MoveArm.createAutoCommand(parsedCommand, arm);
    return driveCommand.alongWith(armCommand);
  }

  public static  Command driveAndGrip(ParsedCommand pc, SwerveDrive swerveDrive, Gripper gripper){
    Command driveCommand = DriveToTarget.createAutoCommand(pc, swerveDrive);
    Command gripCommand = new Grip(gripper);
    // TODO update when we can detect that we have pick up the game piece
    return new ParallelRaceGroup(driveCommand, gripCommand);
  }

  public static Command DriveAndIntake(ParsedCommand pc, SwerveDrive swerveDrive, Gripper gripper, Arm arm) {
    Command driveCommand = DriveToTarget.createAutoCommand(pc, swerveDrive);
    Command gripCommand = new Grip(gripper);
    Command armPoseCommand = MoveArm.createAutoCommand(pc, arm);
    return armPoseCommand.alongWith(new ParallelRaceGroup(driveCommand, gripCommand));
  }
}
