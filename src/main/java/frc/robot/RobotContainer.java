// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.DriverRelativeAngleDrive;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.DriverRelativeSetAngleDrive;
import frc.robot.commands.MoveArm;
import frc.robot.commands.RecalibrateModules;
import frc.robot.commands.ResetHeading;
import frc.robot.commands.ResetPose;
import frc.robot.commands.RobotRelativeDrive;
import frc.robot.commands.SwerveTune;
import frc.robot.commands.SwerveXMode;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Wrist.CoordinateType;
import frc.robot.subsystems.swerve.SwerveDrive;

import com.chaos131.auto.AutoBuilder;
import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

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
  private Arm m_arm = new Arm();
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
    driverControls();
    //m_driver.a().whileTrue(new DriveToTarget(m_swerveDrive, 8, 4, Rotation2d.fromDegrees(90)));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  private void driverControls() {
    m_swerveDrive.setDefaultCommand(new DriverRelativeDrive(m_swerveDrive, m_driver));

    m_driver.povUp().onTrue(new ResetHeading(m_swerveDrive, Rotation2d.fromDegrees(0)));
    m_driver.povDown().onTrue(new ResetHeading(m_swerveDrive, Rotation2d.fromDegrees(180)));
    m_driver.povLeft().onTrue(new ResetHeading(m_swerveDrive, Rotation2d.fromDegrees(90)));
    m_driver.povRight().onTrue(new ResetHeading(m_swerveDrive, Rotation2d.fromDegrees(270)));

    m_driver.a().whileTrue(new SwerveTune(m_swerveDrive));
    m_driver.x().whileTrue(new SwerveXMode(m_swerveDrive));
    m_driver.y().onTrue(new DriverRelativeAngleDrive(m_swerveDrive, m_driver));
    
    m_driver.start().onTrue(new DriverRelativeDrive(m_swerveDrive, m_driver));
    m_driver.back().onTrue(new RobotRelativeDrive(m_swerveDrive, m_driver));

    m_driver.leftBumper().whileTrue(new DriverRelativeSetAngleDrive(m_swerveDrive, m_driver, Rotation2d.fromDegrees(90), 1.0));
    m_driver.leftTrigger().whileTrue(new DriverRelativeSetAngleDrive(m_swerveDrive, m_driver, Rotation2d.fromDegrees(-90), 1.0));
    m_driver.rightBumper().whileTrue(new MoveArm(m_arm, Rotation2d.fromDegrees(-45), ExtenderConstants.MaximumPositionMeters, Rotation2d.fromDegrees(0), CoordinateType.ArmRelative));
    m_driver.rightTrigger().whileTrue(new MoveArm(m_arm, Rotation2d.fromDegrees(-135), ExtenderConstants.MaximumPositionMeters, Rotation2d.fromDegrees(0), CoordinateType.ArmRelative));
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
