// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class DrivePose {
    private static final double FieldWidthFromCADMeters = 16.522;
    private static final double FieldMidpointMeters = FieldWidthFromCADMeters / 2;

    public static Map<String, DrivePose> DrivePoses = new HashMap<String, DrivePose>();

    public final Pose2d m_redPose;
    public final Pose2d m_bluePose;
    public final String m_redName;
    public final String m_blueName;

    private DrivePose(String name, double redXMeters, double redYMeters, Rotation2d redAngle) {
        m_redPose = new Pose2d(redXMeters, redYMeters, redAngle);
        m_bluePose = mirrorPoseOnField(m_redPose);
        m_redName = name + "_RED";
        m_blueName = name + "_BLUE";
        DrivePoses.put(name, this);
    }

    public Pose2d getCurrentAlliancePose() {
        return DriverStation.getAlliance() == Alliance.Blue ? m_bluePose : m_redPose;
    }

    // Score Positions (from red perspective)
    private static final double ScoreXMeters = 1.855;
    private static final Rotation2d ScoreAngle = Rotation2d.fromDegrees(180);
    public static final DrivePose Score1 = new DrivePose("Score1", ScoreXMeters, 7.503, ScoreAngle);
    public static final DrivePose Score2 = new DrivePose("Score2", ScoreXMeters, 6.932, ScoreAngle);
    public static final DrivePose Score3 = new DrivePose("Score3", ScoreXMeters, 6.386, ScoreAngle);
    public static final DrivePose Score4 = new DrivePose("Score4", ScoreXMeters, 5.827, ScoreAngle);
    public static final DrivePose Score5 = new DrivePose("Score5", ScoreXMeters, 5.268, ScoreAngle);
    public static final DrivePose Score6 = new DrivePose("Score6", ScoreXMeters, 4.709, ScoreAngle);
    public static final DrivePose Score7 = new DrivePose("Score7", ScoreXMeters, 4.151, ScoreAngle);
    public static final DrivePose Score8 = new DrivePose("Score8", ScoreXMeters, 3.592, ScoreAngle);
    public static final DrivePose Score9 = new DrivePose("Score9", ScoreXMeters, 3.033, ScoreAngle);

    // Balance  Positions (from red perspective)
    private static final double BalanceYMeters = 5.246;
    private static final Rotation2d BalanceAngle = ScoreAngle;
    public static final DrivePose BalancePrep = new DrivePose("BalancePrep", 2.381, BalanceYMeters, BalanceAngle);
    public static final DrivePose Balance = new DrivePose("Balance", 3.902, BalanceYMeters, BalanceAngle);
    public static final DrivePose BalanceCross = new DrivePose("BalanceCross", 5.500, BalanceYMeters, BalanceAngle);

    // Intake  Positions (from red perspective)
    private static final double IntakePrepXMeters = 6.000;
    private static final double IntakeSweepXMeters = 6.600;
    private static final double Intake1YMeters = 7.100;
    private static final double Intake2YMeters = 5.863;
    private static final double Intake3YMeters = 4.650;
    private static final double Intake4YMeters = 3.472;
    private static final Rotation2d IntakeAngle = ScoreAngle;
    public static final DrivePose IntakePrep1 = new DrivePose("IntakePrep1", IntakePrepXMeters, Intake1YMeters, IntakeAngle);
    public static final DrivePose IntakePrep2 = new DrivePose("IntakePrep2", IntakePrepXMeters, Intake2YMeters, IntakeAngle);
    public static final DrivePose IntakePrep3 = new DrivePose("IntakePrep3", IntakePrepXMeters, Intake3YMeters, IntakeAngle);
    public static final DrivePose IntakePrep4 = new DrivePose("IntakePrep4", IntakePrepXMeters, Intake4YMeters, IntakeAngle);
    public static final DrivePose IntakeSweep1 = new DrivePose("IntakeSweep1", IntakeSweepXMeters, Intake1YMeters, IntakeAngle);
    public static final DrivePose IntakeSweep2 = new DrivePose("IntakeSweep2", IntakeSweepXMeters, Intake2YMeters, IntakeAngle);
    public static final DrivePose IntakeSweep3 = new DrivePose("IntakeSweep3", IntakeSweepXMeters, Intake3YMeters, IntakeAngle);
    public static final DrivePose IntakeSweep4 = new DrivePose("IntakeSweep4", IntakeSweepXMeters, Intake4YMeters, IntakeAngle);


    public static Pose2d getClosestPose(Pose2d robotPose) {
        var poses = new ArrayList<Pose2d>();
        DrivePoses.forEach((name, pose) -> poses.add(pose.getCurrentAlliancePose()));;
        return robotPose.nearest(poses);
    }

    public static Pose2d toCurrentAlliancePose(Pose2d redPose) {
        if(DriverStation.getAlliance() == Alliance.Blue) {
            return mirrorPoseOnField(redPose);
        }
        return redPose;
    }

    public static Pose2d mirrorPoseOnField(Pose2d pose) {
        var xDistanceToMid = FieldMidpointMeters - pose.getX();
        var newX = FieldMidpointMeters + xDistanceToMid;
        var newY = pose.getY();
        var newAngle = pose.getRotation().plus(Rotation2d.fromDegrees(180));
        return new Pose2d(newX, newY, newAngle);
    }
}