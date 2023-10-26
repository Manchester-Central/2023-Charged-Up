// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.chaos131.auto.AutoBuilder;
import com.chaos131.auto.ParsedCommand;
import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AutoBalanceDrive;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DriveUntilTipped;
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
import frc.robot.commands.SetPoseFromLimelights;
import frc.robot.commands.ShuffleBoardPose;
import frc.robot.commands.SwerveXMode;
import frc.robot.commands.UnGrip;
import frc.robot.commands.auto.AutoComboCommands;
import frc.robot.commands.auto.AutoTImerCommand;
import frc.robot.commands.test.TestWrist;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.Gripper;
import frc.robot.subsystems.arm.SmartArmPoseSelector;
import frc.robot.subsystems.arm.Gripper.GripperMode;
import frc.robot.subsystems.arm.SmartArmPoseSelector.PoseType;
import frc.robot.subsystems.swerve.DrivePose;
import frc.robot.subsystems.swerve.SwerveDrive2023;
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
  public SwerveDrive2023 m_swerveDrive = SwerveDrive2023.CreateSwerveDrive(m_limelightLeft, m_limelightRight);
  public LEDs m_leds;
  //private Limelight m_Limelight2 = new Limelight("limeLight2");
  public final Gripper m_gripper = new Gripper();
  public final Arm m_arm = new Arm(m_gripper);
  
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Gamepad m_driver = new Gamepad(OperatorConstants.kDriverControllerPort);

  private final Gamepad m_operator = new Gamepad(OperatorConstants.kOperatorControllerPort);

  public enum ArmMode{ 
    Cube("#8a2be2"),
    Cone("#f9e909"),
    Unset("#134122");
    String m_colorString;
    ArmMode(String colorString){
      m_colorString = colorString;
    }

    public String getColor(){
      return m_colorString;
    }
   }

  private ArmMode m_currentArmMode = ArmMode.Unset;


  private final AutoBuilder autoBuilder = new AutoBuilder();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_leds = new LEDs(() -> m_currentArmMode, m_gripper::hasPiece);
    // Register auto commands
    autoBuilder.registerCommand("resetPosition", (parsedCommand) -> ResetPose.createAutoCommand(parsedCommand, m_swerveDrive));
    autoBuilder.registerCommand("resetPositionWithLimelights", (parsedCommand) -> new SetPoseFromLimelights(m_swerveDrive));
    autoBuilder.registerCommand("drive", (parsedCommand) -> DriveToTarget.createAutoCommand(parsedCommand, m_swerveDrive));
    autoBuilder.registerCommand("xMode", (parsedCommand) -> new SwerveXMode(m_swerveDrive));
    autoBuilder.registerCommand("driveWithLimelights", (parsedCommand) -> DriveToTargetWithLimelights.createAutoCommand(parsedCommand, m_swerveDrive));
    autoBuilder.registerCommand("autoBalance", (parsedCommand) -> new AutoBalanceDrive(m_swerveDrive));
    autoBuilder.registerCommand("moveArm", (parsedCommand) -> MoveArm.createAutoCommand(parsedCommand, m_arm));
    autoBuilder.registerCommand("driveAndMoveArm", (parsedCommand) -> AutoComboCommands.driveAndMoveArm(parsedCommand, m_swerveDrive, m_arm));
    autoBuilder.registerCommand("driveAndGrip", (parsedCommand) -> AutoComboCommands.driveAndGrip(parsedCommand, m_swerveDrive, m_gripper));
    autoBuilder.registerCommand("stow", (parsedCommand) -> new MoveArm(m_arm, ArmPose.StowedPose));
    autoBuilder.registerCommand("score", (parsedCommand) -> Score.createAutoCommand(parsedCommand, m_arm, m_gripper));
    autoBuilder.registerCommand("driveUntilTipped", (parsedCommand)-> DriveUntilTipped.createAutoCommand(parsedCommand, m_swerveDrive));
    autoBuilder.registerCommand("recalibrateArm", (parsedCommand) -> new InstantCommand(() -> m_arm.recalibrateSensors()));
    autoBuilder.registerCommand("driveAndIntake", (parsedCommand) -> AutoComboCommands.DriveAndIntake(parsedCommand, m_swerveDrive, m_gripper, m_arm));
    // Configure the trigger bindings
    configureBindings();
    addCoachTabDashboardValues();
  }

  public void robotPeriodic(){
    AutoBalanceDrive.PIDTuner.tune();
  }

  public void delayedRobotInit(){
    m_swerveDrive.recalibrateModules();
    m_arm.recalibrateSensors();
  }

  public void addCoachTabDashboardValues() {
    var coachTab = Shuffleboard.getTab("Coach");
    m_arm.addCoachTabDashboardValues(coachTab);
    m_gripper.addCoachTabDashboardValues(coachTab);
    m_swerveDrive.addCoachTabDashboardValues(coachTab);
    coachTab.addString("OperatorMode", () -> m_currentArmMode.name());
    coachTab.addString("OperatorModeColor", () -> m_currentArmMode.getColor());
    coachTab.addString("AllianceColor", () -> DriverStation.getAlliance().name());
    coachTab.addString("LeftLimelight", () -> "http://10.1.31.11:5800");
    coachTab.addString("RightLimelight", () -> "http://10.1.31.23:5800");
    Robot.logManager.addNumber("ShoulderOffsetDegrees", true, () -> ArmPose.shoulderOffset_deg);
    // TODO: Figure out the values the coach/drive team want displayed on the dashboard always
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
    var slowModeCommand = new StartEndCommand(()-> {
      SwerveDrive2023.TranslationSpeedModifier = 0.4;
      SwerveDrive2023.RotationSpeedModifier = 0.4;
    }, ()-> {
      SwerveDrive2023.TranslationSpeedModifier = 1;
      SwerveDrive2023.RotationSpeedModifier = 1;
    });
    var creepModeCommand = new StartEndCommand(()-> {
      SwerveDrive2023.TranslationSpeedModifier = 0.2;
      SwerveDrive2023.RotationSpeedModifier = 0.3;
    }, ()-> {
      SwerveDrive2023.TranslationSpeedModifier = 1;
      SwerveDrive2023.RotationSpeedModifier = 1;
    });

    m_driver.start().onTrue(driverRelativeDrive);
    m_driver.back().onTrue(new RobotRelativeDrive(m_swerveDrive, m_driver));

    m_driver.povUp().onTrue(new ResetHeading(m_swerveDrive, DriveDirection.Away));
    m_driver.povDown().onTrue(new ResetHeading(m_swerveDrive, DriveDirection.Towards));
    m_driver.povLeft().onTrue(new ResetHeading(m_swerveDrive, DriveDirection.Left));
    m_driver.povRight().onTrue(new ResetHeading(m_swerveDrive, DriveDirection.Right));
      
    m_driver.leftBumper().whileTrue(new SwerveXMode(m_swerveDrive));
    m_driver.leftTrigger().whileTrue(slowModeCommand);

    m_driver.rightBumper().whileTrue(creepModeCommand);
    m_driver.rightTrigger().whileTrue(new RunCommand(() -> m_gripper.setGripperMode(GripperMode.unGrip), m_gripper));

    m_driver.leftStick().whileTrue(slowModeCommand);
    m_driver.rightStick().whileTrue(creepModeCommand);

    m_driver.a().whileTrue(new DriverRelativeSetAngleDrive(m_swerveDrive, m_driver, DriveDirection.Towards, 1.0));
    m_driver.b().whileTrue(new DriverRelativeSetAngleDrive(m_swerveDrive, m_driver, DriveDirection.Right, 1.0));
    m_driver.x().whileTrue(new DriverRelativeSetAngleDrive(m_swerveDrive, m_driver, DriveDirection.Left, 1.0));
    m_driver.y().whileTrue(new DriverRelativeSetAngleDrive(m_swerveDrive, m_driver, DriveDirection.Away, 1.0));
  }

  private void operaterControls(){
    m_arm.setDefaultCommand(new DefaultArmCommand(m_arm));
    Command defaultGripCommand = new InstantCommand(() -> m_gripper.setGripperMode(GripperMode.hold), m_gripper);
    m_gripper.setDefaultCommand(defaultGripCommand);

    // Pose selection
    m_operator.y().onTrue(new InstantCommand(() -> m_currentArmMode = ArmMode.Cone));
    m_operator.x().onTrue(new InstantCommand(() -> m_currentArmMode = ArmMode.Cube));
    m_operator.x().whileTrue(new StartEndCommand(() -> LEDs.flashing = true, () -> LEDs.flashing = false));
    m_operator.y().whileTrue(new StartEndCommand(() -> LEDs.flashing = true, () -> LEDs.flashing = false));
    
    m_operator.a().whileTrue(new Grip(m_gripper));
    m_operator.b().whileTrue(new UnGrip(m_gripper));

    BooleanSupplier isConeMode = () -> m_currentArmMode == ArmMode.Cone;
    BooleanSupplier isCubeMode = () -> m_currentArmMode == ArmMode.Cube;

    m_operator.povUp().debounce(0.1, DebounceType.kFalling).and(isConeMode).whileTrue(scorePrep(ArmPose.ConeHighPose, ArmPose.ConeHighPoseBack));
    m_operator.povUp().debounce(0.1, DebounceType.kFalling).and(isCubeMode).whileTrue(scorePrep(ArmPose.CubeHighPose, ArmPose.CubeHighPoseBack));
    
    m_operator.povLeft().debounce(0.1, DebounceType.kFalling).and(isConeMode).whileTrue(scorePrep(ArmPose.ConeMidPose, ArmPose.ConeMidPoseBack));
    m_operator.povLeft().debounce(0.1, DebounceType.kFalling).and(isCubeMode).whileTrue(scorePrep(ArmPose.CubeMidPose, ArmPose.CubeMidPoseBack));
    
    m_operator.povDown().debounce(0.1, DebounceType.kFalling).whileTrue(scorePrep(ArmPose.LowScorePose, ArmPose.LowScorePoseBack));
    
    m_operator.povRight().debounce(0.1, DebounceType.kFalling).whileTrue(scorePrep(ArmPose.LowScorePoseBack, ArmPose.LowScorePoseBack));

    // Intake Controls
    m_operator.leftBumper().debounce(0.1, DebounceType.kFalling).and(isConeMode).whileTrue(intake(ArmPose.IntakeDoubleStationConeFront, ArmPose.IntakeDoubleStationConeBack, PoseType.doubleSub));
    m_operator.leftBumper().debounce(0.1, DebounceType.kFalling).and(isCubeMode).whileTrue(intake(ArmPose.IntakeDoubleStationCubeFront, ArmPose.IntakeDoubleStationCubeBack, PoseType.doubleSub));
    m_operator.leftTrigger().debounce(0.1, DebounceType.kFalling).and(isConeMode).whileTrue(intake(ArmPose.IntakeConeVerticalBack));
    m_operator.leftTrigger().debounce(0.1, DebounceType.kFalling).and(isCubeMode).whileTrue(intake(ArmPose.IntakeCubeBack));

    
    m_operator.rightBumper().debounce(0.1, DebounceType.kFalling).and(isConeMode).whileTrue(intake(ArmPose.IntakeSingleStationConeFront, ArmPose.IntakeSingleStationConeBack, PoseType.singleSub));
    m_operator.rightBumper().debounce(0.1, DebounceType.kFalling).and(isCubeMode).whileTrue(intake(ArmPose.IntakeSingleStationCubeFront, ArmPose.IntakeSingleStationCubeBack, PoseType.singleSub));
    m_operator.rightTrigger().debounce(0.1, DebounceType.kFalling).and(isConeMode).whileTrue(intake(ArmPose.IntakeConeTippedBack));
    m_operator.rightTrigger().debounce(0.1, DebounceType.kFalling).and(isCubeMode).whileTrue(intake(ArmPose.IntakeCubeBack));
    
    // test
    // if (DebugConstants.EnableArmDebug) {
    //   m_operator.start().whileTrue(new ShuffleBoardPose(m_arm, "start").repeatedly());
    //   m_operator.back().whileTrue(new ShuffleBoardPose(m_arm, "back").repeatedly());
    // }
    // m_operator.back().onTrue(new InstantCommand(() -> m_swerveDrive.recalibrateModules()));  
    // m_operator.start().onTrue(new InstantCommand(() -> m_arm.recalibrateSensors()));
    m_operator.back().onTrue(new InstantCommand(() -> ArmPose.shoulderOffset_deg -= 1));
    m_operator.start().onTrue(new InstantCommand(() -> ArmPose.shoulderOffset_deg += 1));
    // m_operator.back().whileTrue(new RunCommand(() -> m_swerveDrive.updatePoseFromLimelights()));
  }

  private Command scorePrep(ArmPose frontPose, ArmPose backPose) {
    SmartArmPoseSelector poseSelector = new SmartArmPoseSelector(frontPose, backPose, PoseType.score);
    return new MoveArm(m_arm, () -> poseSelector.getSmartPose(m_swerveDrive.getOdometryRotation())).repeatedly();
  }

  private Command intake(ArmPose pose) {
    return new MoveArm(m_arm, pose).repeatedly().alongWith(new Grip(m_gripper));
  }

  private Command intake(ArmPose frontPose, ArmPose backPose, PoseType poseType) {
    SmartArmPoseSelector poseSelector = new SmartArmPoseSelector(frontPose, backPose, poseType);
    return new MoveArm(m_arm, () -> poseSelector.getSmartPose(m_swerveDrive.getOdometryRotation())).repeatedly().alongWith(new Grip(m_gripper));
  }

  private void dashboardCommands() {
    // created a test command on Shuffleboard for each known pose
    if (DebugConstants.EnableArmDebug) {
      ArmPose.forAllPoses((String poseName, ArmPose pose) -> SmartDashboard.putData("Set Arm Pose/" + poseName, new MoveArm(m_arm, pose).repeatedly()));
    }
    if (DebugConstants.EnableDriveDebug) {
      DrivePose.DrivePoses.forEach((String poseName, DrivePose pose) -> {
        SmartDashboard.putData("Drive To Target/" + pose.m_redName, new DriveToTarget(m_swerveDrive, pose.m_redPose, SwerveConstants.DriveToTargetTolerance, SwerveConstants.MaxTranslationPIDSpeedPercent));
        SmartDashboard.putData("Drive To Target/" + pose.m_blueName, new DriveToTarget(m_swerveDrive, pose.m_bluePose, SwerveConstants.DriveToTargetTolerance, SwerveConstants.MaxTranslationPIDSpeedPercent));
      });
    }
  }

  private void testCommands() {
    // Don't make the tester controller and commands if not in debug mode
    if(!DebugConstants.IsDebugMode) {
      return;
    }

    Gamepad testController = new Gamepad(OperatorConstants.kTesterControllerPort);
    // testController.a().whileTrue(new MoveExtender(m_arm, ExtenderConstants.MinimumPositionMeters + 0.02));
    // testController.x().whileTrue(new MoveExtender(m_arm, 1.1));
    // testController.y().whileTrue(new MoveExtender(m_arm, ExtenderConstants.MaximumPositionMeters - 0.02));
    // testController.b().whileTrue(new TestWrist(m_arm, testController));
    // testController.back().whileTrue(new Grip(m_gripper));
    // testController.start().whileTrue(new UnGrip(m_gripper));
    // testController.povUp().whileTrue(new MoveShoulder(m_arm, Rotation2d.fromDegrees(0)));
    // testController.povRight().whileTrue(new MoveShoulder(m_arm, Rotation2d.fromDegrees(-45)));
    // testController.povDown().whileTrue(new MoveShoulder(m_arm, Rotation2d.fromDegrees(-90)));
    // testController.povLeft().whileTrue(new MoveShoulder(m_arm, Rotation2d.fromDegrees(-135)));
    // testController.rightTrigger().whileTrue(new MoveWrist(m_arm, Rotation2d.fromDegrees(90)));
    // testController.rightBumper().whileTrue(new MoveWrist(m_arm, Rotation2d.fromDegrees(270)));
    // testController.leftBumper().whileTrue(new MoveWrist(m_arm, Rotation2d.fromDegrees(180)));
    // testController.a().whileTrue(new RunCommand( () -> m_gripper.setGripperMode(GripperMode.grip),m_gripper));
    // testController.b().whileTrue(new RunCommand( () -> m_gripper.setGripperMode(GripperMode.hold),m_gripper));
    // testController.y().whileTrue(new RunCommand( () -> m_gripper.setGripperMode(GripperMode.unGrip),m_gripper));
    // testController.povUp().whileTrue(new ShuffleBoardPose(m_arm, "povUp").repeatedly());
    // testController.povDown().whileTrue(new ShuffleBoardPose(m_arm, "povDown").repeatedly());
    // testController.rightTrigger().whileTrue(new AutoBalanceDrive(m_swerveDrive));
    // testController.rightBumper().whileTrue(
    //   new DriveToTargetWithLimelights(m_swerveDrive, () -> DrivePose.Balance.getCurrentAlliancePose(), Constants.DriveToTargetTolerance)
    //   .andThen(new SwerveXMode(m_swerveDrive))
    // );
    var xStart = 8;
    var yStart = 4;
    testController.start().onTrue(new ResetPose(m_swerveDrive, new Pose2d(xStart, yStart, Rotation2d.fromDegrees(0))));
    testController.b().whileTrue(new DriveToTarget(m_swerveDrive, new Pose2d(xStart, yStart, Rotation2d.fromDegrees(0)), SwerveConstants.DriveToTargetTolerance, SwerveConstants.MaxTranslationPIDSpeedPercent));
    testController.y().whileTrue(new DriveToTarget(m_swerveDrive, new Pose2d(xStart + 1, yStart, Rotation2d.fromDegrees(90)), SwerveConstants.DriveToTargetTolerance, SwerveConstants.MaxTranslationPIDSpeedPercent));
    testController.a().whileTrue(new DriveToTarget(m_swerveDrive, new Pose2d(xStart - 1, yStart, Rotation2d.fromDegrees(270)), SwerveConstants.DriveToTargetTolerance, SwerveConstants.MaxTranslationPIDSpeedPercent));
    testController.x().whileTrue(new DriveToTarget(m_swerveDrive, new Pose2d(xStart, yStart + 1, Rotation2d.fromDegrees(180)), SwerveConstants.DriveToTargetTolerance, SwerveConstants.MaxTranslationPIDSpeedPercent));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Always stow the arm before any auto
    return new InstantCommand(() -> m_gripper.setGripperMode(GripperMode.hold)).andThen(autoBuilder.createAutoCommand().deadlineWith(new AutoTImerCommand()));
  }

  public void addSmartDashboard() {
    if (DebugConstants.EnableDriveDebug) {
      SmartDashboard.putNumber("Driver Left X", m_driver.getLeftX());
      SmartDashboard.putNumber("Driver Right X", m_driver.getRightX());
      SmartDashboard.putNumber("Driver Left Y", m_driver.getLeftY());
      SmartDashboard.putNumber("Driver Right Y", m_driver.getRightY());
    }
  }
}
