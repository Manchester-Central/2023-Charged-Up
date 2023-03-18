// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class SafetyZoneHelper {
    private double m_minimum;
    private double m_maximum;
    private double m_excludeMin;
    private double m_excludeMax;

    public SafetyZoneHelper(double minimum, double maximum) {
        m_minimum = minimum;
        m_maximum = maximum;
        m_excludeMin = Double.NaN;
        m_excludeMax = Double.NaN;
    }
    public double getSafeValue(double startingValue) {
        
        double clampValue =  MathUtil.clamp(startingValue, m_minimum, m_maximum);
        boolean isExcludeMinSet = Double.isFinite(m_excludeMin);
        boolean isExcludeMaxSet = Double.isFinite(m_excludeMax);
        if (!isExcludeMinSet && !isExcludeMaxSet){
            return clampValue;
        }

        if (isExcludeMinSet && !isExcludeMaxSet){
            return Math.min(clampValue, m_excludeMin);            
        }
        if (!isExcludeMinSet && isExcludeMaxSet){
            return Math.max(clampValue, m_excludeMax);
        }
        if (clampValue > m_excludeMin && clampValue < m_excludeMax){
            double diffup = Math.abs(clampValue - m_excludeMax);
            double diffdown = Math.abs(clampValue - m_excludeMin); 

            if (diffup < diffdown){
                return m_excludeMax; 
            }
            else {
                return m_excludeMin;
            }
        }     
        return clampValue;
    }
    public void exclude(double excludeMin, double excludeMax){
        m_excludeMin = excludeMin;
        m_excludeMax = excludeMax;
    }
    public void excludeUp(double excludeMin){
        exclude(excludeMin, Double.NaN);
    }
    public void excludeDown(double excludeMax){
        exclude(Double.NaN, excludeMax);
    }
    public void resetToDefault(){
        m_excludeMax = Double.NaN;
        m_excludeMin = Double.NaN;
    }
        
}

// Code for safety zone assistance used by the robot's arm. Sets the maximum and minimum parameters for the robot to avoid crashing into the electronics on the base. 