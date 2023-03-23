// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.auto.AutoBuilder;
import com.chaos131.auto.ParsedCommand;
import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoBalanceDrive;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.DriveToTargetWithLimelights;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.DriverRelativeSetAngleDrive;
import frc.robot.commands.Grip;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveExtender;
import frc.robot.commands.MoveShoulder;
import frc.robot.commands.MoveWrist;
import frc.robot.commands.ResetHeading;
import frc.robot.commands.ResetPose;
import frc.robot.commands.RobotRelativeDrive;
import frc.robot.commands.Score;
import frc.robot.commands.ShuffleBoardPose;
import frc.robot.commands.SwerveXMode;
import frc.robot.commands.UnGrip;
import frc.robot.commands.auto.AutoComboCommands;
import frc.robot.commands.auto.AutoTImerCommand;
import frc.robot.commands.test.TestWrist;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.Gripper;
import frc.robot.subsystems.arm.Gripper.GripperMode;
import frc.robot.subsystems.swerve.DrivePose;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.DriveDirection;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private Limelight m_limelightLeft = new Limelight("limelight-left");
  private Limelight m_limelightRight = new Limelight ("limelight-right");
  private SwerveDrive m_swerveDrive = new SwerveDrive(m_limelightLeft, m_limelightRight);
  //private Limelight m_Limelight2 = new Limelight("limeLight2");
  public final Gripper m_gripper = new Gripper();
  public final Arm m_arm = new Arm(m_gripper);
  
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Gamepad m_driver = new Gamepad(OperatorConstants.kDriverControllerPort);

  private final Gamepad m_operator = new Gamepad(OperatorConstants.kOperatorControllerPort);

  private final Gamepad m_tester = new Gamepad(OperatorConstants.kTesterControllerPort);

  private enum ArmMode{ 
    Cube("#8a2be2"),
    Cone("#f9e909"),
    Intake("#134122");
    String m_colorString;
    ArmMode(String colorString){
      m_colorString = colorString;
    }

    public String getColor(){
      return m_colorString;
    }
   }

  private ArmMode m_currentArmMode = ArmMode.Intake;


  private final AutoBuilder autoBuilder = new AutoBuilder();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Register auto commands
    autoBuilder.registerCommand("resetPosition", (parsedCommand) -> ResetPose.createAutoCommand(parsedCommand, m_swerveDrive));
    autoBuilder.registerCommand("resetPositionWithLimelights", (parsedCommand) -> new InstantCommand(() -> m_swerveDrive.updatePoseFromLimelights(), m_swerveDrive));
    autoBuilder.registerCommand("drive", (parsedCommand) -> DriveToTarget.createAutoCommand(parsedCommand, m_swerveDrive));
    autoBuilder.registerCommand("xMode", (parsedCommand) -> new SwerveXMode(m_swerveDrive));
    autoBuilder.registerCommand("driveWithLimelights", (parsedCommand) -> DriveToTargetWithLimelights.createAutoCommand(parsedCommand, m_swerveDrive));
    autoBuilder.registerCommand("autoBalance", (parsedCommand) -> new AutoBalanceDrive(m_swerveDrive));
    autoBuilder.registerCommand("moveArm", (parsedCommand) -> MoveArm.createAutoCommand(parsedCommand, m_arm));
    autoBuilder.registerCommand("driveAndMoveArm", (parsedCommand) -> AutoComboCommands.driveAndMoveArm(parsedCommand, m_swerveDrive, m_arm));
    autoBuilder.registerCommand("driveAndGrip", (parsedCommand) -> AutoComboCommands.driveAndGrip(parsedCommand, m_swerveDrive, m_gripper));
    autoBuilder.registerCommand("stow", (parsedCommand) -> new MoveArm(m_arm, ArmPose.StowedPose));
    autoBuilder.registerCommand("score", (parsedCommand) -> Score.createAutoCommand(parsedCommand, m_arm, m_gripper));
    // Configure the trigger bindings
    configureBindings();
  }

  
  public void robotPeriodic(){
    SmartDashboard.putString("OperatorMode", m_currentArmMode.name());
    SmartDashboard.putString("OperatorModeColor", m_currentArmMode.getColor());
    AutoBalanceDrive.PIDTuner.tune();
  }

  public void delayedRobotInit(){
    m_swerveDrive.recalibrateModules();
    m_arm.recalibrateSensors();
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
    operaterControls();
    dashboardCommands();
    testCommands();
  }

  private void driverControls() {
    Command driverRelativeDrive = new DriverRelativeDrive(m_swerveDrive, m_driver);
    m_swerveDrive.setDefaultCommand(driverRelativeDrive);
    var slowModeCommand = new StartEndCommand(()-> SwerveDrive.SpeedModifier = 0.4, ()-> SwerveDrive.SpeedModifier = 1);

    m_driver.start().onTrue(driverRelativeDrive);
    m_driver.back().onTrue(new RobotRelativeDrive(m_swerveDrive, m_driver));

    m_driver.povUp().onTrue(new ResetHeading(m_swerveDrive, DriveDirection.Away));
    m_driver.povDown().onTrue(new ResetHeading(m_swerveDrive, DriveDirection.Towards));
    m_driver.povLeft().onTrue(new ResetHeading(m_swerveDrive, DriveDirection.Left));
    m_driver.povRight().onTrue(new ResetHeading(m_swerveDrive, DriveDirection.Right));
      
    m_driver.leftBumper().whileTrue(new SwerveXMode(m_swerveDrive));
    m_driver.leftTrigger().whileTrue(slowModeCommand);

    m_driver.rightBumper().whileTrue(slowModeCommand);
    m_driver.rightTrigger().whileTrue(new RunCommand(() -> m_gripper.setGripperMode(GripperMode.unGrip), m_gripper));

    m_driver.leftStick().whileTrue(slowModeCommand);
    m_driver.rightStick().whileTrue(slowModeCommand);

    m_driver.a().whileTrue(new DriverRelativeSetAngleDrive(m_swerveDrive, m_driver, DriveDirection.Towards, 1.0));
    m_driver.b().whileTrue(new DriverRelativeSetAngleDrive(m_swerveDrive, m_driver, DriveDirection.Right, 1.0));
    m_driver.x().whileTrue(new DriverRelativeSetAngleDrive(m_swerveDrive, m_driver, DriveDirection.Left, 1.0));
    m_driver.y().whileTrue(new DriverRelativeSetAngleDrive(m_swerveDrive, m_driver, DriveDirection.Away, 1.0));
  }

  private void operaterControls(){
    m_arm.setDefaultCommand(new DefaultArmCommand(m_arm, m_tester));
    Command defaultGripCommand = new InstantCommand(() -> m_gripper.setGripperMode(GripperMode.hold), m_gripper);
    m_gripper.setDefaultCommand(defaultGripCommand);

    // Pose selection
    m_operator.y().onTrue(new InstantCommand(()-> m_currentArmMode = ArmMode.Cone));
    m_operator.x().onTrue(new InstantCommand(()-> m_currentArmMode = ArmMode.Cube));
    m_operator.a().whileTrue(new Grip(m_gripper));
    m_operator.b().whileTrue(new UnGrip(m_gripper));

    Command highCone = new MoveArm(m_arm, ArmPose.ConeHighPose).repeatedly();
    Command midCone = new MoveArm(m_arm, ArmPose.ConeMidPose).repeatedly();
    Command highCube = new MoveArm(m_arm, ArmPose.CubeHighPose).repeatedly();
    Command midCube = new MoveArm(m_arm, ArmPose.CubeMidPose).repeatedly();
    Command lowPose = new MoveArm(m_arm, ArmPose.LowScorePose).repeatedly();

    m_operator.povUp().and(()-> m_currentArmMode == ArmMode.Cone).whileTrue(highCone);
    m_operator.povUp().and(()-> m_currentArmMode == ArmMode.Cube).whileTrue(highCube);
    
    m_operator.povLeft().and(()-> m_currentArmMode == ArmMode.Cone).whileTrue(midCone);
    m_operator.povLeft().and(()-> m_currentArmMode == ArmMode.Cube).whileTrue(midCube);
    
    m_operator.povDown().and(()-> m_currentArmMode == ArmMode.Cone).whileTrue(lowPose);
    m_operator.povDown().and(()-> m_currentArmMode == ArmMode.Cube).whileTrue(lowPose);
    
    m_operator.povRight().whileTrue(new MoveArm(m_arm, ArmPose.LowScorePoseBack));

    // Intake Controls
    m_operator.leftBumper().whileTrue(new MoveArm(m_arm, ArmPose.IntakeDoubleStation));
    m_operator.leftTrigger().and(()-> m_currentArmMode == ArmMode.Cone).whileTrue(
      new MoveArm(m_arm, ArmPose.IntakeConeVerticalBack).repeatedly()
      .alongWith(new Grip(m_gripper)));
    m_operator.leftTrigger().and(()-> m_currentArmMode == ArmMode.Cube).whileTrue(new MoveArm(m_arm, ArmPose.IntakeCubeBack).repeatedly().alongWith(new Grip(m_gripper)));

    
    m_operator.rightBumper().whileTrue(new MoveArm(m_arm, ArmPose.IntakeSingleStation));
    m_operator.rightTrigger().and(()-> m_currentArmMode == ArmMode.Cone).whileTrue(new MoveArm(m_arm, ArmPose.IntakeConeTippedBack).repeatedly().alongWith(new Grip(m_gripper)));
    m_operator.rightTrigger().and(()-> m_currentArmMode == ArmMode.Cube).whileTrue(new MoveArm(m_arm, ArmPose.IntakeCubeBack).repeatedly().alongWith(new Grip(m_gripper)));
    
    // test
    m_operator.start().whileTrue(new ShuffleBoardPose(m_arm, "start").repeatedly());
    m_operator.back().whileTrue(new ShuffleBoardPose(m_arm, "back").repeatedly());
  }

  private ArmPose armPoseTarget;

  private void dashboardCommands() {
    // created a test command on Shuffleboard for each known pose
    ArmPose.forAllPoses((String poseName, ArmPose pose) -> SmartDashboard.putData("Set Arm Pose/" + poseName, new MoveArm(m_arm, pose).repeatedly()));
    DrivePose.DrivePoses.forEach((String poseName, DrivePose pose) -> {
      SmartDashboard.putData("Drive To Target/" + pose.m_redName, new DriveToTarget(m_swerveDrive, pose.m_redPose, Constants.DriveToTargetTolerance));
      SmartDashboard.putData("Drive To Target/" + pose.m_blueName, new DriveToTarget(m_swerveDrive, pose.m_bluePose, Constants.DriveToTargetTolerance));
    });
    ArmPose.forAllPoses((String poseName, ArmPose pose) -> SmartDashboard.putData("ArmTarget/" + poseName, new InstantCommand(() -> {
      armPoseTarget = pose;
      SmartDashboard.putNumber("PoseTest/povUp/Shoulder", pose.shoulderAngle.getDegrees());
      SmartDashboard.putNumber("PoseTest/povUp/Extender", pose.extenderPos);
      SmartDashboard.putNumber("PoseTest/povUp/Wrist", pose.wristAngle.getDegrees());
    })));
  }

  private void testCommands() {
    // m_tester.a().whileTrue(new MoveExtender(m_arm, ExtenderConstants.MinimumPositionMeters + 0.02));
    // m_tester.x().whileTrue(new MoveExtender(m_arm, 1.1));
    // m_tester.y().whileTrue(new MoveExtender(m_arm, ExtenderConstants.MaximumPositionMeters - 0.02));
    // m_tester.b().whileTrue(new TestWrist(m_arm, m_tester));
    // m_tester.back().whileTrue(new Grip(m_gripper));
    // m_tester.start().whileTrue(new UnGrip(m_gripper));
    // m_tester.povUp().whileTrue(new MoveShoulder(m_arm, Rotation2d.fromDegrees(0)));
    // m_tester.povRight().whileTrue(new MoveShoulder(m_arm, Rotation2d.fromDegrees(-45)));
    // m_tester.povDown().whileTrue(new MoveShoulder(m_arm, Rotation2d.fromDegrees(-90)));
    // m_tester.povLeft().whileTrue(new MoveShoulder(m_arm, Rotation2d.fromDegrees(-135)));
    // m_tester.rightTrigger().whileTrue(new MoveWrist(m_arm, Rotation2d.fromDegrees(90)));
    // m_tester.rightBumper().whileTrue(new MoveWrist(m_arm, Rotation2d.fromDegrees(270)));
    // m_tester.leftBumper().whileTrue(new MoveWrist(m_arm, Rotation2d.fromDegrees(180)));
    m_tester.a().whileTrue(new RunCommand( () -> m_gripper.setGripperMode(GripperMode.grip),m_gripper));
    m_tester.b().whileTrue(new RunCommand( () -> m_gripper.setGripperMode(GripperMode.hold),m_gripper));
    m_tester.y().whileTrue(new RunCommand( () -> m_gripper.setGripperMode(GripperMode.unGrip),m_gripper));
    m_tester.povUp().whileTrue(new ShuffleBoardPose(m_arm, "povUp").repeatedly());
    m_tester.povDown().whileTrue(new ShuffleBoardPose(m_arm, "povDown").repeatedly());
    m_tester.rightTrigger().whileTrue(new AutoBalanceDrive(m_swerveDrive));
    m_tester.rightBumper().whileTrue(
      new DriveToTargetWithLimelights(m_swerveDrive, () -> DrivePose.Balance.getCurrentAlliancePose(), Constants.DriveToTargetTolerance)
      .andThen(new SwerveXMode(m_swerveDrive))
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Always stow the arm before any auto
    return (new MoveArm(m_arm, ArmPose.StowedPose).andThen(autoBuilder.createAutoCommand())).deadlineWith(new AutoTImerCommand());
  }

  public void addSmartDashboard() {
    SmartDashboard.putNumber("Driver Left X", m_driver.getLeftX());
    SmartDashboard.putNumber("Driver Right X", m_driver.getRightX());
    SmartDashboard.putNumber("Driver Left Y", m_driver.getLeftY());
    SmartDashboard.putNumber("Driver Right Y", m_driver.getRightY());
  }
}
