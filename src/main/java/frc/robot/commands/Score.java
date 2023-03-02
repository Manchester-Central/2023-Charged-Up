// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import javax.sql.CommonDataSource;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPose;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score extends SequentialCommandGroup {
  /** Creates a new Score. */
  public Score(Arm arm, Supplier<ArmPose> prepPoseSupplier, Supplier<ArmPose> targetPoseSupplier) {
    if (prepPoseSupplier != null){
      addCommands(new MoveArm(arm, prepPoseSupplier));
    }
    Command goToScorePosition = new MoveArm(arm, targetPoseSupplier);
    Command release = new UnGrip(arm).withTimeout(1);
    addCommands(goToScorePosition, release);
    if (prepPoseSupplier != null){
      addCommands(new MoveArm(arm, prepPoseSupplier));
    }
  }

  public Score(Arm arm, Supplier<ArmPose> targetPoseSupplier) {
    this (arm, null, targetPoseSupplier);
  }

  public Score(Arm arm, ArmPose pose) {
    this(arm, () -> pose);
  }
}
