// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.MoveArm;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.swerve.DrivePose;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * A class for combining other commands into single auto commands
 */
public class AutoComboCommands {
  
  public static Command driveAndMoveArm(ParsedCommand parsedCommand, SwerveDrive swerveDrive, Arm arm) {
    String drivePoseName = parsedCommand.getArgument("drivePose");
    DrivePose drivePose = drivePoseName == null ? null : DrivePose.DrivePoses.get(drivePoseName);

    String armPoseName = parsedCommand.getArgument("armPose");
    ArmPose armPose = armPoseName == null ? null : ArmPose.ArmPoses.get(armPoseName);

    if(armPose == null || drivePose == null) {
      return new InstantCommand();
    }

    var driveCommand = new DriveToTarget(swerveDrive, drivePose.getCurrentAlliancePose(), Constants.DriveToTargetTolerance);
    var armCommand = new MoveArm(arm, armPose);
    return driveCommand.alongWith(armCommand);
  }
}
