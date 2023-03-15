// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class ScorePose {
    private static final double Row1YMeters = 7.503;
    private static final double Row2YMeters = 6.932;
    private static final double Row3YMeters = 6.386;
    private static final double Row4YMeters = 5.827;
    private static final double Row5YMeters = 5.268;
    private static final double Row6YMeters = 4.709;
    private static final double Row7YMeters = 4.151;
    private static final double Row8YMeters = 3.592;
    private static final double Row9YMeters = 3.033;


    private static final double RedXMeters = 1.855;
    private static final Rotation2d RedAngle = Rotation2d.fromDegrees(180);
    
    private static final double BlueXMeters = 14.657;
    private static final Rotation2d BlueAngle = Rotation2d.fromDegrees(0);


    public static Map<String, Pose2d> ScorePoses = new HashMap<String, Pose2d>();
    private static Pose2d addStaticPose(String name, double x_meters, double y_meters, Rotation2d angle) {
        Pose2d pose = new Pose2d(x_meters, y_meters, angle);
        ScorePoses.put(name, pose);
        return pose;
    }

    // Red Alliance
    public static final Pose2d Red1 = addStaticPose("Red1", RedXMeters, Row1YMeters, RedAngle);
    public static final Pose2d Red2 = addStaticPose("Red2", RedXMeters, Row2YMeters, RedAngle);
    public static final Pose2d Red3 = addStaticPose("Red3", RedXMeters, Row3YMeters, RedAngle);
    public static final Pose2d Red4 = addStaticPose("Red4", RedXMeters, Row4YMeters, RedAngle);
    public static final Pose2d Red5 = addStaticPose("Red5", RedXMeters, Row5YMeters, RedAngle);
    public static final Pose2d Red6 = addStaticPose("Red6", RedXMeters, Row6YMeters, RedAngle);
    public static final Pose2d Red7 = addStaticPose("Red7", RedXMeters, Row7YMeters, RedAngle);
    public static final Pose2d Red8 = addStaticPose("Red8", RedXMeters, Row8YMeters, RedAngle);
    public static final Pose2d Red9 = addStaticPose("Red9", RedXMeters, Row9YMeters, RedAngle);

    // Blue Alliance
    public static final Pose2d Blue1 = addStaticPose("Blue1", BlueXMeters, Row1YMeters, BlueAngle);
    public static final Pose2d Blue2 = addStaticPose("Blue2", BlueXMeters, Row2YMeters, BlueAngle);
    public static final Pose2d Blue3 = addStaticPose("Blue3", BlueXMeters, Row3YMeters, BlueAngle);
    public static final Pose2d Blue4 = addStaticPose("Blue4", BlueXMeters, Row4YMeters, BlueAngle);
    public static final Pose2d Blue5 = addStaticPose("Blue5", BlueXMeters, Row5YMeters, BlueAngle);
    public static final Pose2d Blue6 = addStaticPose("Blue6", BlueXMeters, Row6YMeters, BlueAngle);
    public static final Pose2d Blue7 = addStaticPose("Blue7", BlueXMeters, Row7YMeters, BlueAngle);
    public static final Pose2d Blue8 = addStaticPose("Blue8", BlueXMeters, Row8YMeters, BlueAngle);
    public static final Pose2d Blue9 = addStaticPose("Blue9", BlueXMeters, Row9YMeters, BlueAngle);


    public static Pose2d getClosestPose(Pose2d robotPose) {
        return robotPose.nearest(new ArrayList<Pose2d>(ScorePoses.values()));
    }
}