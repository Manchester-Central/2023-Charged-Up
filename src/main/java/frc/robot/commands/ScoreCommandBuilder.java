package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPose;

public class ScoreCommandBuilder extends CommandBase {
    public ScoreCommandBuilder(Command goToScorePosition, int timeS, Command commandQueue, Arm arm) {
        Command stow = new MoveArm(arm, ArmPose.StowedPose);
        Command release = new UnGrip(arm).withTimeout(timeS);
        commandQueue = stow.andThen(goToScorePosition).andThen(release).andThen(stow);
      }
}
