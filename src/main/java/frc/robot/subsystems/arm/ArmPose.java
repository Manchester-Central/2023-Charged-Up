// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.subsystems.arm.Wrist.CoordinateType;

/** Add your docs here. */
public class ArmPose {
    public final Rotation2d shoulderAngle;
    public final double extenderPos;
    public final Rotation2d wristAngle;
    public final CoordinateType wristCoordinate;

    public ArmPose(Rotation2d shoulderAngle, double extenderPos, Rotation2d wristAngle, CoordinateType wristCoordinate){
        this.shoulderAngle = shoulderAngle;
        this.extenderPos = extenderPos;
        this.wristAngle = wristAngle;
        this.wristCoordinate = wristCoordinate;
    }

    public ArmPose(double shoulderAngleDegrees, double extenderPos, double wristAngleDegrees, CoordinateType wristCoordinate){
        this(Rotation2d.fromDegrees(shoulderAngleDegrees), extenderPos, Rotation2d.fromDegrees(wristAngleDegrees), wristCoordinate);
    }

    public static Map<String, ArmPose> ArmPoses = new HashMap<String, ArmPose>();
    private static ArmPose addStaticPose(String name, double shoulderAngleDegrees, double extenderPos, double wristAngleDegrees, CoordinateType wristCoordinate) {
        var pose = new ArmPose(shoulderAngleDegrees, extenderPos, wristAngleDegrees, wristCoordinate);
        ArmPoses.put(name, pose);
        return pose;
    }

    //Stowed Pose
    public static final ArmPose StowedPose = addStaticPose(
        "Stowed", 
        -95,
        ExtenderConstants.MinimumPositionMeters, 
        190, 
        CoordinateType.ArmRelative
    );

    //Intake Poses
    public static final ArmPose IntakeConeVerticalBack = addStaticPose(
        "IntakeConeVerticalBack", 
        -125, 
        ExtenderConstants.MinimumPositionMeters, 
        336, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose IntakeConeTippedBack = addStaticPose(
        "IntakeConeTippedBack", 
        -126, 
        1.040, 
        329, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose IntakeCubeBack = addStaticPose(
        "IntakeCubeBack", 
        -121, 
        0.803, 
        343, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose IntakeConeVerticalFront = addStaticPose(
        "IntakeConeVerticalFront", 
        -65, 
        ExtenderConstants.MinimumPositionMeters, 
        40, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose IntakeConeTippedFront = addStaticPose(
        "IntakeConeTippedFront", 
        -67, 
        0.95, 
        35, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose IntakeCubeFloorFront = addStaticPose(
        "IntakeCubeFloorFront", 
        -70, 
        ExtenderConstants.MinimumPositionMeters, 
        33, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose IntakeSingleStation = addStaticPose(
        "IntakeSingleStation", 
        -101.2, 
        ExtenderConstants.MinimumPositionMeters, 
        215.5, 
        CoordinateType.ArmRelative
    );

    // Low pose
    public static final ArmPose LowScorePose = addStaticPose(
        "LowScorePose", 
        -86, 
        ExtenderConstants.MinimumPositionMeters, 
        105, 
        CoordinateType.ArmRelative
    );

    //Cube Poses
    public static final ArmPose CubeMidPose = addStaticPose(
        "CubeMidPose", 
        -36, 
        ExtenderConstants.MinimumPositionMeters, 
        78, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose CubeHighPose = addStaticPose(
        "CubeHighPose", 
        -11, 
        1.03, 
        50, 
        CoordinateType.ArmRelative
    );

    //Cone Poses

    public static final ArmPose ConeMidPose = addStaticPose(
        "ConeMidPose", 
        0, 
        ExtenderConstants.MinimumPositionMeters, 
        311, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose ConeHighPose = addStaticPose(
        "ConeHighPose", 
        9.4, 
        1.194, 
        321, 
        CoordinateType.ArmRelative
    );

    // Pick stations
    public static final ArmPose DoublePickPose = addStaticPose(
        "DoublePickPose", 
        -7, 
        0.8, 
        10, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose SinglePickPose = addStaticPose(
        "SinglePickPose", 
        -50, 
        0.8, 
        90, 
        CoordinateType.ArmRelative
    );

    //Test Poses
    public static final ArmPose TopRightTestPose = addStaticPose(
        "TopRight", 
        0, 
        ExtenderConstants.MaximumPositionMeters, 
        180, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose TopLeftTestPose = addStaticPose(
        "TopLeft", 
        -180, 
        ExtenderConstants.MaximumPositionMeters, 
        180, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose BottomRightTestPose = addStaticPose(
        "BottomRight", 
        -45, 
        ExtenderConstants.MinimumPositionMeters, 
        180, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose BottomLeftTestPose = addStaticPose(
        "BottomLeft", 
        -135, 
        ExtenderConstants.MinimumPositionMeters, 
        180, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose StraightPose = addStaticPose(
        "Straight", 
        0, 
        ExtenderConstants.MaximumPositionMeters, 
        180, 
        CoordinateType.ArmRelative
    );



    public static void forAllPoses(BiConsumer<String, ArmPose> lambdaFunction) {
        ArmPoses.forEach(lambdaFunction);
    }
}