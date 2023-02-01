// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.RecalibrateModules;
import frc.robot.commands.ResetPose;
import frc.robot.commands.RobotRelativeDrive;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;

import com.chaos131.auto.AutoBuilder;
import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import com.chaos131.gamepads.Gamepad;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private SwerveDrive m_swerveDrive = new SwerveDrive();
  private Limelight m_Limelight = new Limelight();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Gamepad m_driver = new Gamepad(OperatorConstants.kDriverControllerPort);

  private final Gamepad m_operator = new Gamepad(OperatorConstants.kOperatorControllerPort);


  private final AutoBuilder autoBuilder = new AutoBuilder();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Register auto commands
    autoBuilder.registerCommand("resetPosition", (ParsedCommand pc) -> ResetPose.createAutoCommand(pc, m_swerveDrive));
    autoBuilder.registerCommand("driveToTarget", (ParsedCommand pc) -> DriveToTarget.createAutoCommand(pc, m_swerveDrive));
    // Configure the trigger bindings
    configureBindings();
  }
  public void delayedRobotInit(){
    m_swerveDrive.recalibrateModules();
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_swerveDrive.setDefaultCommand(new RobotRelativeDrive(m_swerveDrive, m_driver));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driver.b().whileTrue(new RunCommand(() -> m_driver.getHID().setRumble(RumbleType.kBothRumble, 0.5)));
    m_driver.b().whileFalse(new RunCommand(() -> m_driver.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
    Shuffleboard.getTab("Test").addBoolean("b pressed", () -> m_driver.b().getAsBoolean());
    m_driver.a().whileTrue(new DriveToTarget(m_swerveDrive, 8, 4, Rotation2d.fromDegrees(90)));
    m_driver.povUp().onTrue(new ResetPose(m_swerveDrive, new Pose2d(8, 4, Rotation2d.fromDegrees(0))));
    m_driver.povDown().onTrue(new ResetPose(m_swerveDrive, new Pose2d(8, 4, Rotation2d.fromDegrees(180))));
    m_driver.povRight().onTrue(new RecalibrateModules(m_swerveDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoBuilder.createAutoCommand();
  }

  public void addSmartDashboard() {
    SmartDashboard.putNumber("Driver Left X", m_driver.getLeftX());
    SmartDashboard.putNumber("Driver Right X", m_driver.getRightX());
    SmartDashboard.putNumber("Driver Left Y", m_driver.getLeftY());
    SmartDashboard.putNumber("Driver Right Y", m_driver.getRightY());
  }
}
