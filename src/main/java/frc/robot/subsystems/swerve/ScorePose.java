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
    private static final double Row1YMeters = 5.0;
    private static final double Row2YMeters = 4.45;
    private static final double Row3YMeters = 3.95;
    private static final double Row4YMeters = 3.45;
    private static final double Row5YMeters = 2.95;
    private static final double Row6YMeters = 2.45;
    private static final double Row7YMeters = 1.95;
    private static final double Row8YMeters = 1.45;
    private static final double Row9YMeters = 0.98;


    private static final double RedXMeters = 13.4;
    private static final Rotation2d RedAngle = Rotation2d.fromDegrees(0);
    
    private static final double BlueXMeters = 2.5;
    private static final Rotation2d BlueAngle = Rotation2d.fromDegrees(180);


    public static Map<String, Pose2d> ScorePoses = new HashMap<String, Pose2d>();
    private static Pose2d addStaticPose(String name, double x_meters, double y_meters, Rotation2d angle) {
        Pose2d pose = new Pose2d(x_meters, y_meters, angle);
        ScorePoses.put(name, pose);
        return pose;
    }

    // Red Alliance
    public static final Pose2d Red1Cone = addStaticPose("Red1Cone", RedXMeters, Row1YMeters, RedAngle);
    public static final Pose2d Red2Cube = addStaticPose("Red2Cube", RedXMeters, Row2YMeters, RedAngle);
    public static final Pose2d Red3Cone = addStaticPose("Red3Cone", RedXMeters, Row3YMeters, RedAngle);
    public static final Pose2d Red4Cone = addStaticPose("Red4Cone", RedXMeters, Row4YMeters, RedAngle);
    public static final Pose2d Red5Cube = addStaticPose("Red5Cube", RedXMeters, Row5YMeters, RedAngle);
    public static final Pose2d Red6Cone = addStaticPose("Red6Cone", RedXMeters, Row6YMeters, RedAngle);
    public static final Pose2d Red7Cone = addStaticPose("Red7Cone", RedXMeters, Row7YMeters, RedAngle);
    public static final Pose2d Red8Cube = addStaticPose("Red8Cube", RedXMeters, Row8YMeters, RedAngle);
    public static final Pose2d Red9Cone = addStaticPose("Red9Cone", RedXMeters, Row9YMeters, RedAngle);

    // Blue Alliance
    public static final Pose2d Blue1Cone = addStaticPose("Blue1Cone", BlueXMeters, Row1YMeters, BlueAngle);
    public static final Pose2d Blue2Cube = addStaticPose("Blue2Cube", BlueXMeters, Row2YMeters, BlueAngle);
    public static final Pose2d Blue3Cone = addStaticPose("Blue3Cone", BlueXMeters, Row3YMeters, BlueAngle);
    public static final Pose2d Blue4Cone = addStaticPose("Blue4Cone", BlueXMeters, Row4YMeters, BlueAngle);
    public static final Pose2d Blue5Cube = addStaticPose("Blue5Cube", BlueXMeters, Row5YMeters, BlueAngle);
    public static final Pose2d Blue6Cone = addStaticPose("Blue6Cone", BlueXMeters, Row6YMeters, BlueAngle);
    public static final Pose2d Blue7Cone = addStaticPose("Blue7Cone", BlueXMeters, Row7YMeters, BlueAngle);
    public static final Pose2d Blue8Cube = addStaticPose("Blue8Cube", BlueXMeters, Row8YMeters, BlueAngle);
    public static final Pose2d Blue9Cone = addStaticPose("Blue9Cone", BlueXMeters, Row9YMeters, BlueAngle);


    public static Pose2d getClosestPose(Pose2d robotPose) {
        return robotPose.nearest(new ArrayList<Pose2d>(ScorePoses.values()));
    }
}