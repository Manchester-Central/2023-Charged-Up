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
    public static final double shoulderOffset = 0;

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
        -93 + shoulderOffset,
        ExtenderConstants.MinimumPositionMeters, 
        190, 
        CoordinateType.ArmRelative
    );

    //Intake Poses
    public static final ArmPose IntakeConeVerticalBack = addStaticPose(
        "IntakeConeVerticalBack", 
        -123 + shoulderOffset, 
        ExtenderConstants.MinimumPositionMeters, 
        336, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose IntakeConeTippedBack = addStaticPose(
        "IntakeConeTippedBack", 
        -124 + shoulderOffset, 
        1.040, 
        329, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose IntakeCubeBack = addStaticPose(
        "IntakeCubeBack", 
        -119 + shoulderOffset, 
        0.803, 
        343, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose IntakeConeVerticalFront = addStaticPose(
        "IntakeConeVerticalFront", 
        -63 + shoulderOffset, 
        ExtenderConstants.MinimumPositionMeters, 
        40, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose IntakeConeTippedFront = addStaticPose(
        "IntakeConeTippedFront", 
        -65 + shoulderOffset, 
        0.95, 
        35, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose IntakeCubeFloorFront = addStaticPose(
        "IntakeCubeFloorFront", 
        -68 + shoulderOffset, 
        ExtenderConstants.MinimumPositionMeters, 
        33, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose IntakeSingleStationCubeFront = addStaticPose(
        "IntakeSingleStationCubeFront", 
        -86.8 + shoulderOffset, 
        ExtenderConstants.MinimumPositionMeters, 
        144.5, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose IntakeSingleStationCubeBack = addStaticPose(
        "IntakeSingleStationCubeBack", 
        -99.2 + shoulderOffset, 
        ExtenderConstants.MinimumPositionMeters, 
        215.5, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose IntakeSingleStationConeFront = addStaticPose(
        "IntakeSingleStationConeFront", 
        -86.8 + shoulderOffset, 
        ExtenderConstants.MinimumPositionMeters, 
        135, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose IntakeSingleStationConeBack = addStaticPose(
        "IntakeSingleStationConeBack", 
        -99.2 + shoulderOffset, 
        ExtenderConstants.MinimumPositionMeters, 
        225.0, 
        CoordinateType.ArmRelative
    );

    public static final ArmPose IntakeDoubleStationConeFront = addStaticPose(
        "IntakeDoubleStationConeFront", 
        -0.43 + shoulderOffset,
        0.823, 
        350.01, 
        CoordinateType.ArmRelative
    ); 

    public static final ArmPose IntakeDoubleStationCubeFront = addStaticPose(
        "IntakeDoubleStationCubeFront", 
        10.82 + shoulderOffset,
        0.818, 
        317.51, 
        CoordinateType.ArmRelative
    ); 

    public static final ArmPose IntakeDoubleStationConeBack = addStaticPose(
        "IntakeDoubleStationConeBack", 
        -186.13 + shoulderOffset,
        0.82, 
        28.8, 
        CoordinateType.ArmRelative
    ); 

    public static final ArmPose IntakeDoubleStationCubeBack = addStaticPose(
        "IntakeDoubleStationCubeBack", 
        167.30 + shoulderOffset,
        0.874, 
        45, 
        CoordinateType.ArmRelative
    ); 

    // Low pose
    public static final ArmPose LowScorePose = addStaticPose(
        "LowScorePose", 
        -84 + shoulderOffset, 
        ExtenderConstants.MinimumPositionMeters, 
        105, 
        CoordinateType.ArmRelative
    );

    public static final ArmPose LowScorePoseBack = addStaticPose(
        "LowScorePoseBack", 
        -97 + shoulderOffset, 
        ExtenderConstants.MinimumPositionMeters, 
        255, 
        CoordinateType.ArmRelative
    ); // TODO find correct values

    //Cube Poses
    public static final ArmPose CubeMidPose = addStaticPose(
        "CubeMidPose", 
        7 + shoulderOffset, 
        ExtenderConstants.MinimumPositionMeters, 
        311, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose CubeMidPoseBack = addStaticPose(
        "CubeMidPoseBack", 
        -193 + shoulderOffset, 
        ExtenderConstants.MinimumPositionMeters, 
        49, 
        CoordinateType.ArmRelative //TODO finalize values
    );
    public static final ArmPose CubeHighPose = addStaticPose(
        "CubeHighPose", 
        16.4 + shoulderOffset,
        1.194, 
        321, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose CubeHighPoseBack = addStaticPose(
        "CubeHighPoseBack", 
        -202.4 + shoulderOffset, 
        1.194, 
        39, 
        CoordinateType.ArmRelative //TODO finalize values
    );

    //Cone Poses

    public static final ArmPose ConeMidPose = addStaticPose(
        "ConeMidPose", 
        2 + shoulderOffset, 
        ExtenderConstants.MinimumPositionMeters, 
        311, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose ConeMidPoseBack = addStaticPose(
        "ConeMidPoseBack", 
        -188 + shoulderOffset, 
        ExtenderConstants.MinimumPositionMeters, 
        49, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose ConeHighPose = addStaticPose(
        "ConeHighPose", 
        13.5 + shoulderOffset, 
        1.194, 
        321, 
        CoordinateType.ArmRelative
    );
    public static final ArmPose ConeHighPoseBack = addStaticPose(
        "ConeHighPoseBack", 
        -199.5 + shoulderOffset, 
        1.194, 
        39, 
        CoordinateType.ArmRelative
    );

    // Pick stations

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