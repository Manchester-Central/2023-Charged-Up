// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.auto.AutoBuilder;
import com.chaos131.auto.ParsedCommand;
import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.DriverRelativeAngleDrive;
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
import frc.robot.commands.SwerveTune;
import frc.robot.commands.SwerveXMode;
import frc.robot.commands.UnGrip;
import frc.robot.commands.test.TestExtender;
import frc.robot.commands.test.TestShoulder;
import frc.robot.commands.test.TestWrist;
import frc.robot.subsystems.ArduinoIO;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.Gripper.GripperMode;
import frc.robot.subsystems.arm.Wrist.CoordinateType;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final boolean EnableTesterController = false;
  // The robot's subsystems and commands are defined here...

  private SwerveDrive m_swerveDrive = new SwerveDrive();
  private Limelight m_Limelight = new Limelight("limeLight");
  private Limelight m_Limelight2 = new Limelight("limeLight2");
  public final Arm m_arm = new Arm();
  private ArduinoIO m_arduinoIO = new ArduinoIO();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Gamepad m_driver = new Gamepad(OperatorConstants.kDriverControllerPort);

  private final Gamepad m_operator = new Gamepad(OperatorConstants.kOperatorControllerPort);

  private ArmPose m_nextPrepPose = ArmPose.StowedPose;
  private ArmPose m_nextScorePose = ArmPose.StowedPose;
  private ArmPose m_nextIntakePose = ArmPose.IntakeFront;
  private enum ArmMode{ 
    Cube,
    Cone,
    Intake;
   }

  private ArmMode m_currentArmMode = ArmMode.Intake;


  private final AutoBuilder autoBuilder = new AutoBuilder();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Register auto commands
    autoBuilder.registerCommand("resetPosition", (ParsedCommand pc) -> ResetPose.createAutoCommand(pc, m_swerveDrive));
    autoBuilder.registerCommand("driveToTarget", (ParsedCommand pc) -> DriveToTarget.createAutoCommand(pc, m_swerveDrive));
    autoBuilder.registerCommand("namedPose", (ParsedCommand pc) -> MoveArm.createAutoCommand(pc, m_arm));
    // Configure the trigger bindings
    configureBindings();
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
    //m_driver.a().whileTrue(new DriveToTarget(m_swerveDrive, 8, 4, Rotation2d.fromDegrees(90)));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  private void driverControls() {
    m_swerveDrive.setDefaultCommand(new DriverRelativeDrive(m_swerveDrive, m_driver));
    // m_swerveDrive.setDefaultCommand(new RobotRelativeDrive(m_swerveDrive, m_driver));

    m_driver.povUp().onTrue(new ResetHeading(m_swerveDrive, Rotation2d.fromDegrees((DriverStation.getAlliance() == Alliance.Red) ? 180:0)));
    m_driver.povDown().onTrue(new ResetHeading(m_swerveDrive, Rotation2d.fromDegrees((DriverStation.getAlliance() == Alliance.Red) ? 360:180)));
    m_driver.povLeft().onTrue(new ResetHeading(m_swerveDrive, Rotation2d.fromDegrees((DriverStation.getAlliance() == Alliance.Red) ? 270:90)));
    m_driver.povRight().onTrue(new ResetHeading(m_swerveDrive, Rotation2d.fromDegrees((DriverStation.getAlliance() == Alliance.Red) ? 90:270)));
      

    // // Practice score commands - should move targets to operator
    m_driver.rightTrigger().whileTrue(new Score(m_arm, () -> m_nextPrepPose, () -> m_nextScorePose, 1));

    m_driver.leftBumper().whileTrue(new SwerveXMode(m_swerveDrive));
    m_driver.leftTrigger().whileTrue(new StartEndCommand(()-> SwerveDrive.SpeedModifier = 0.5, ()-> SwerveDrive.SpeedModifier = 1));
    m_driver.leftStick().whileTrue(new InstantCommand(()->m_arm.setGripperMode(GripperMode.grip)).andThen(new MoveArm(m_arm, ArmPose.IntakeBack).repeatedly()));
    // m_driver.y().onTrue(new DriverRelativeAngleDrive(m_swerveDrive, m_driver));
    
    // m_driver.start().onTrue(new DriverRelativeDrive(m_swerveDrive, m_driver));
    // m_driver.back().onTrue(new RobotRelativeDrive(m_swerveDrive, m_driver));

    // m_driver.leftBumper().whileTrue(new DriverRelativeSetAngleDrive(m_swerveDrive, m_driver, Rotation2d.fromDegrees(90), 1.0));
    // m_driver.leftTrigger().whileTrue(new DriverRelativeSetAngleDrive(m_swerveDrive, m_driver, Rotation2d.fromDegrees(-90), 1.0));
  }

  private void operaterControls(){
    m_arm.setDefaultCommand(new DefaultArmCommand(m_arm));

    // // Grip/Ungrip
    // m_operator.rightBumper().whileTrue(new UnGrip(m_arm));
    // m_operator.rightTrigger().whileTrue(new Grip(m_arm));

    // // Cubes
    // m_operator.a().whileTrue(new MoveArm(m_arm, ArmPose.LowScorePose).repeatedly());
    // m_operator.x().whileTrue(new MoveArm(m_arm, ArmPose.CubeMidPose).repeatedly());
    // m_operator.y().whileTrue(new MoveArm(m_arm, ArmPose.CubeHighPose).repeatedly());
    m_operator.y().onTrue(new InstantCommand(()-> m_currentArmMode = ArmMode.Cone));
    m_operator.x().onTrue(new InstantCommand(()-> m_currentArmMode = ArmMode.Cube));
    m_operator.a().onTrue(new InstantCommand(()-> m_currentArmMode = ArmMode.Intake));
    m_operator.b().toggleOnTrue(new RunCommand(()-> m_arm.stop(), m_arm));

    // Cone poses
    m_operator.povUp().and(()-> m_currentArmMode == ArmMode.Cone).whileTrue(createScorePrep(ArmPose.ConeHighPosePrep, ArmPose.ConeHighPose));
    m_operator.povLeft().and(()-> m_currentArmMode == ArmMode.Cone).whileTrue(createScorePrep(ArmPose.ConeMidPosePrep, ArmPose.ConeMidPose));
    m_operator.povDown().and(()-> m_currentArmMode == ArmMode.Cone).whileTrue(createScorePrep(ArmPose.LowScorePose));
      
    // Cone poses
    m_operator.povUp().and(()-> m_currentArmMode == ArmMode.Cube).whileTrue(createScorePrep(ArmPose.CubeHighPose));
    m_operator.povLeft().and(()-> m_currentArmMode == ArmMode.Cube).whileTrue(createScorePrep(ArmPose.CubeMidPose));
    m_operator.povDown().and(()-> m_currentArmMode == ArmMode.Cube).whileTrue(createScorePrep(ArmPose.LowScorePose));
    
    // Intake poses
    m_operator.povUp().and(()-> m_currentArmMode == ArmMode.Intake).whileTrue(createIntakePrep(ArmPose.DoublePickPose));
    m_operator.povLeft().and(()-> m_currentArmMode == ArmMode.Intake).whileTrue(createIntakePrep(ArmPose.SinglePickPose));
    m_operator.povDown().and(()-> m_currentArmMode == ArmMode.Intake).whileTrue(createIntakePrep(ArmPose.IntakeFront));
    
    m_operator.povRight().whileTrue(new MoveArm(m_arm, ArmPose.StowedPose));

    m_operator.rightBumper().whileTrue(new InstantCommand(()->m_arm.setGripperMode(GripperMode.slowGrip)).andThen(new MoveArm(m_arm, () -> m_nextIntakePose).repeatedly()));
    m_operator.rightTrigger().whileTrue(new InstantCommand(()->m_arm.setGripperMode(GripperMode.grip)).andThen(new MoveArm(m_arm, () -> m_nextIntakePose).repeatedly()));

    m_operator.leftBumper().whileTrue(new InstantCommand(()->m_arm.setGripperMode(GripperMode.slowUngrip), m_arm).repeatedly());
    m_operator.leftTrigger().whileTrue(new InstantCommand(()->m_arm.setGripperMode(GripperMode.unGrip), m_arm).repeatedly());

    // temp. Replace with LEDS when working 
    m_operator.back().whileTrue(new Score(m_arm, () -> m_nextPrepPose, () -> m_nextScorePose, 1));

    // // Cones
    // m_operator.povDown().whileTrue(new MoveArm(m_arm, ArmPose.LowScorePose).repeatedly());
    // m_operator.povRight().whileTrue(new MoveArm(m_arm, ArmPose.ConeMidPose).repeatedly());
    // m_operator.povUp().whileTrue(new MoveArm(m_arm, ArmPose.ConeHighPose).repeatedly());

    // // Intakes
    // m_operator.povLeft().whileTrue(new MoveArm(m_arm, ArmPose.IntakeBake).andThen(new Grip(m_arm)));
    // m_operator.b().whileTrue(new MoveArm(m_arm, ArmPose.IntakeFront).andThen(new Grip(m_arm)));

    // // Pickups
    // m_operator.leftTrigger().whileTrue(new MoveArm(m_arm, ArmPose.DoublePickPose).andThen(new Grip(m_arm)));
    // m_operator.leftBumper().whileTrue(new MoveArm(m_arm, ArmPose.SinglePickPose).andThen(new Grip(m_arm)));

    
    // // test
    // m_operator.start().whileTrue(new ShuffleBoardPose(m_arm).repeatedly());
  }

  public Command createIntakePrep(ArmPose intakePose) {
    return new InstantCommand(() -> {
      m_nextIntakePose = intakePose;
    }).andThen(new MoveArm(m_arm, intakePose)).repeatedly();
  }

  public Command createScorePrep(ArmPose scorePose) {
    return createScorePrep(scorePose, scorePose);
  }

  public Command createScorePrep(ArmPose prepPose, ArmPose scorePose) {
    return new InstantCommand(() -> {
      m_nextPrepPose = prepPose;
      m_nextScorePose = scorePose;
    }).andThen(new MoveArm(m_arm, prepPose)).repeatedly();
  }

  private void dashboardCommands() {
    // created a test command on Shuffleboard for each known pose (waits 2 seconds because the ChaosBoard needs to be in focus to run correctly)
    ArmPose.forAllPoses((String poseName, ArmPose pose) -> SmartDashboard.putData("Set Arm Pose/" + poseName, new WaitCommand(2).andThen(new MoveArm(m_arm, pose))));
  }

  private void testCommands() {
    // only enable the tester when needed, so we don't have joystick unplugged errors
    if(EnableTesterController) {
      Gamepad tester = new Gamepad(OperatorConstants.kTesterControllerPort);
      tester.a().whileTrue(new MoveExtender(m_arm, ExtenderConstants.MinimumPositionMeters + 0.02));
      tester.x().whileTrue(new MoveExtender(m_arm, 1.1));
      tester.y().whileTrue(new MoveExtender(m_arm, ExtenderConstants.MaximumPositionMeters - 0.02));
      tester.b().whileTrue(new TestWrist(m_arm, tester));
      tester.back().whileTrue(new Grip(m_arm));
      tester.start().whileTrue(new UnGrip(m_arm));
      tester.povUp().whileTrue(new MoveShoulder(m_arm, Rotation2d.fromDegrees(0)));
      tester.povRight().whileTrue(new MoveShoulder(m_arm, Rotation2d.fromDegrees(-45)));
      tester.povDown().whileTrue(new MoveShoulder(m_arm, Rotation2d.fromDegrees(-90)));
      tester.povLeft().whileTrue(new MoveShoulder(m_arm, Rotation2d.fromDegrees(-135)));
      tester.rightTrigger().whileTrue(new MoveWrist(m_arm, Rotation2d.fromDegrees(90)));
      tester.rightBumper().whileTrue(new MoveWrist(m_arm, Rotation2d.fromDegrees(270)));
      tester.leftBumper().whileTrue(new MoveWrist(m_arm, Rotation2d.fromDegrees(180)));
    }
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
    // SmartDashboard.putNumber("Driver Left X", m_driver.getLeftX());
    // SmartDashboard.putNumber("Driver Right X", m_driver.getRightX());
    // SmartDashboard.putNumber("Driver Left Y", m_driver.getLeftY());
    // SmartDashboard.putNumber("Driver Right Y", m_driver.getRightY());
  }
}
