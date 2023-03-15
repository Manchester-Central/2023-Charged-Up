// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.auto.AutoBuilder;
import com.chaos131.auto.ParsedCommand;
import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.Grip;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveExtender;
import frc.robot.commands.MoveShoulder;
import frc.robot.commands.MoveWrist;
import frc.robot.commands.ResetHeading;
import frc.robot.commands.ResetPose;
import frc.robot.commands.RobotRelativeDrive;
import frc.robot.commands.Score;
import frc.robot.commands.SwerveXMode;
import frc.robot.commands.UnGrip;
import frc.robot.commands.test.TestWrist;
import frc.robot.subsystems.ArduinoIO;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.Gripper;
import frc.robot.subsystems.arm.Gripper.GripperMode;
import frc.robot.subsystems.swerve.ScorePose;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private SwerveDrive m_swerveDrive = new SwerveDrive();
  private Limelight m_limelightLeft = new Limelight("limelight-left");
  private Limelight m_limelightRight = new Limelight ("limelight-right");
  //private Limelight m_Limelight2 = new Limelight("limeLight2");
  public final Gripper m_gripper = new Gripper();
  public final Arm m_arm = new Arm(m_gripper);
  private ArduinoIO m_arduinoIO = new ArduinoIO();
  
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Gamepad m_driver = new Gamepad(OperatorConstants.kDriverControllerPort);

  private final Gamepad m_operator = new Gamepad(OperatorConstants.kOperatorControllerPort);

  private final Gamepad m_tester = new Gamepad(OperatorConstants.kTesterControllerPort);

  private ArmPose m_nextPrepPose = ArmPose.StowedPose;
  private ArmPose m_nextPose = ArmPose.StowedPose;
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
    autoBuilder.registerCommand("driveToScorePose", (ParsedCommand pc) -> DriveToTarget.createAutoCommandForScorePose(pc, m_swerveDrive));
    autoBuilder.registerCommand("namedPose", (ParsedCommand pc) -> MoveArm.createAutoCommand(pc, m_arm));
    autoBuilder.registerCommand("driveAndGrip", this::CreateDriveAndGrip);
    autoBuilder.registerCommand("cubeHighPose", (ParsedCommand) -> new MoveArm(m_arm, ArmPose.CubeHighPose));
    autoBuilder.registerCommand("cubeMidPose", (ParsedCommand) -> new MoveArm(m_arm, ArmPose.CubeMidPose));
    autoBuilder.registerCommand("stow", (ParsedCommand) -> new MoveArm(m_arm, ArmPose.StowedPose));
    autoBuilder.registerCommand("score", (ParsedCommand pc) -> Score.createAutoCommand(pc, m_arm, m_gripper));
    // Configure the trigger bindings
    configureBindings();
  }

  private Command CreateDriveAndGrip(ParsedCommand pc){
    Command driveCommand = DriveToTarget.createAutoCommand(pc, m_swerveDrive);
    Command gripCommand = new Grip(m_gripper);
    // TODO update when we can detect that we have pick up the game piece
    return new ParallelRaceGroup(driveCommand, gripCommand);
  }

  
  public void robotPeriodic(){
    Pose2d LLLeftPose = m_limelightLeft.getPose();
    Pose2d LLRightPose = m_limelightRight.getPose();
    if (LLLeftPose != null && LLRightPose != null){
      System.out.println(LLLeftPose.toString() + LLRightPose.toString()); 
      double leftDistance = Math.abs(m_limelightLeft.getTargetXDistancePixels());
      double rightDistance = Math.abs(m_limelightRight.getTargetXDistancePixels());
      if (leftDistance < rightDistance){
        m_swerveDrive.resetPose(LLLeftPose);
      }
      else{
        m_swerveDrive.resetPose(LLRightPose);
      }
      
    }
    else if (LLRightPose != null){
      System.out.println(LLRightPose.toString()); 

      m_swerveDrive.resetPose(LLRightPose);
    }
    else if (LLLeftPose != null){
      System.out.println(LLLeftPose.toString()); 

      m_swerveDrive.resetPose(LLLeftPose);
    }
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
    m_driver.start().onTrue(driverRelativeDrive);
    m_driver.back().onTrue(new RobotRelativeDrive(m_swerveDrive, m_driver));

    m_driver.povUp().onTrue(new ResetHeading(m_swerveDrive, Rotation2d.fromDegrees((DriverStation.getAlliance() == Alliance.Blue) ? 180:0)));
    m_driver.povDown().onTrue(new ResetHeading(m_swerveDrive, Rotation2d.fromDegrees((DriverStation.getAlliance() == Alliance.Blue) ? 360:180)));
    m_driver.povLeft().onTrue(new ResetHeading(m_swerveDrive, Rotation2d.fromDegrees((DriverStation.getAlliance() == Alliance.Blue) ? 270:90)));
    m_driver.povRight().onTrue(new ResetHeading(m_swerveDrive, Rotation2d.fromDegrees((DriverStation.getAlliance() == Alliance.Blue) ? 90:270)));
      
    m_driver.leftBumper().whileTrue(new SwerveXMode(m_swerveDrive));
    m_driver.leftTrigger().whileTrue(new StartEndCommand(()-> SwerveDrive.SpeedModifier = 0.5, ()-> SwerveDrive.SpeedModifier = 1));
    // NOTE: Driver Right Trigger is used in operaterControls()

    //m_driver.leftStick().whileTrue(new InstantCommand(()->m_arm.setGripperMode(GripperMode.grip)).andThen(new MoveArm(m_arm, ArmPose.IntakeBack).repeatedly()));
    // m_driver.y().onTrue(new DriverRelativeAngleDrive(m_swerveDrive, m_driver));
   
    // m_driver.leftBumper().whileTrue(new DriverRelativeSetAngleDrive(m_swerveDrive, m_driver, Rotation2d.fromDegrees(90), 1.0));
    // m_driver.leftTrigger().whileTrue(new DriverRelativeSetAngleDrive(m_swerveDrive, m_driver, Rotation2d.fromDegrees(-90), 1.0));
    //m_gripperMutex.setDefaultCommand(new InstantCommand( () -> m_arm.setGripperMode(GripperMode.hold), m_gripperMutex ));
    //m_driver.rightBumper().whileTrue(new InstantCommand( () -> m_arm.setGripperMode(GripperMode.grip) ));
    //m_driver.rightTrigger().whileTrue(new InstantCommand( () -> m_arm.setGripperMode(GripperMode.unGrip) ));
  }

  private void operaterControls(){
    m_arm.setDefaultCommand(new DefaultArmCommand(m_arm, m_tester));
    Command defaultGripCommand = new InstantCommand( () -> m_gripper.setGripperMode(GripperMode.grip),m_gripper);
    m_gripper.setDefaultCommand(defaultGripCommand);

    // Pose selection
    m_operator.y().onTrue(new InstantCommand(()-> m_currentArmMode = ArmMode.Cone));
    m_operator.x().onTrue(new InstantCommand(()-> m_currentArmMode = ArmMode.Cube));
    m_operator.a().onTrue(new InstantCommand(()-> m_currentArmMode = ArmMode.Intake));
    m_operator.b().toggleOnTrue(new RunCommand(()-> m_arm.stop(), m_arm));

    Command highCone = new Score(m_arm, m_gripper, ArmPose.ConeHighPose, m_driver.rightTrigger()::getAsBoolean).repeatedly();
    Command midCone = new Score(m_arm, m_gripper, ArmPose.ConeMidPose, m_driver.rightTrigger()::getAsBoolean).repeatedly();
    Command highCube = new Score(m_arm, m_gripper, ArmPose.CubeHighPose, m_driver.rightTrigger()::getAsBoolean).repeatedly();
    Command midCube = new Score(m_arm, m_gripper, ArmPose.CubeMidPose, m_driver.rightTrigger()::getAsBoolean).repeatedly();
    Command lowPose = new Score(m_arm, m_gripper, ArmPose.LowScorePose, m_driver.rightTrigger()::getAsBoolean).repeatedly();

    m_operator.povUp().and(()-> m_currentArmMode == ArmMode.Cone).whileTrue(highCone);
    m_operator.povUp().and(()-> m_currentArmMode == ArmMode.Cube).whileTrue(highCube);
    m_operator.povUp().and(()-> m_currentArmMode == ArmMode.Intake).whileTrue(new MoveArm(m_arm, ArmPose.DoublePickPose).repeatedly());

    m_operator.povLeft().and(()-> m_currentArmMode == ArmMode.Cone).whileTrue(midCone);
    m_operator.povLeft().and(()-> m_currentArmMode == ArmMode.Cube).whileTrue(midCube);
    m_operator.povLeft().and(()-> m_currentArmMode == ArmMode.Intake).whileTrue(new MoveArm(m_arm, ArmPose.SinglePickPose).repeatedly());

    m_operator.povDown().and(()-> m_currentArmMode == ArmMode.Cone).whileTrue(lowPose);
    m_operator.povDown().and(()-> m_currentArmMode == ArmMode.Cube).whileTrue(lowPose);
    m_operator.povDown().and(()-> m_currentArmMode == ArmMode.Intake).whileTrue(new MoveArm(m_arm, ArmPose.IntakeFront).repeatedly());

    m_operator.povRight().whileTrue(new MoveArm(m_arm, ArmPose.StowedPose).repeatedly());
    
    // // test
    // m_operator.start().whileTrue(new ShuffleBoardPose(m_arm).repeatedly());
  }

  private void dashboardCommands() {
    // created a test command on Shuffleboard for each known pose
    ArmPose.forAllPoses((String poseName, ArmPose pose) -> SmartDashboard.putData("Set Arm Pose/" + poseName, new MoveArm(m_arm, pose).repeatedly()));
    ScorePose.ScorePoses.forEach((String poseName, Pose2d pose) -> SmartDashboard.putData("Drive To Target/" + poseName, new DriveToTarget(m_swerveDrive, pose, Constants.DriveToTargetTolerance)));
  }

  private void testCommands() {
    m_tester.a().whileTrue(new MoveExtender(m_arm, ExtenderConstants.MinimumPositionMeters + 0.02));
    m_tester.x().whileTrue(new MoveExtender(m_arm, 1.1));
    m_tester.y().whileTrue(new MoveExtender(m_arm, ExtenderConstants.MaximumPositionMeters - 0.02));
    m_tester.b().whileTrue(new TestWrist(m_arm, m_tester));
    m_tester.back().whileTrue(new Grip(m_gripper));
    m_tester.start().whileTrue(new UnGrip(m_gripper));
    m_tester.povUp().whileTrue(new MoveShoulder(m_arm, Rotation2d.fromDegrees(0)));
    m_tester.povRight().whileTrue(new MoveShoulder(m_arm, Rotation2d.fromDegrees(-45)));
    m_tester.povDown().whileTrue(new MoveShoulder(m_arm, Rotation2d.fromDegrees(-90)));
    m_tester.povLeft().whileTrue(new MoveShoulder(m_arm, Rotation2d.fromDegrees(-135)));
    m_tester.rightTrigger().whileTrue(new MoveWrist(m_arm, Rotation2d.fromDegrees(90)));
    m_tester.rightBumper().whileTrue(new MoveWrist(m_arm, Rotation2d.fromDegrees(270)));
    m_tester.leftBumper().whileTrue(new MoveWrist(m_arm, Rotation2d.fromDegrees(180)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Always stow the arm before any auto
    return new MoveArm(m_arm, ArmPose.StowedPose).andThen(autoBuilder.createAutoCommand());
  }

  public void addSmartDashboard() {
    SmartDashboard.putNumber("Driver Left X", m_driver.getLeftX());
    SmartDashboard.putNumber("Driver Right X", m_driver.getRightX());
    SmartDashboard.putNumber("Driver Left Y", m_driver.getLeftY());
    SmartDashboard.putNumber("Driver Right Y", m_driver.getRightY());
  }
}
