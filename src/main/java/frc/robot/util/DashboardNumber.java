// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class DashboardNumber {
    private static Set<String> ChangedValues = new HashSet<>();
    private static List<DashboardNumber> AllUpdaters = new ArrayList<>();

    private double m_value;
    private String m_name;
    private Consumer<Double> m_onUpdate;
    private boolean m_tuningEnabled;
    
    public DashboardNumber(String name, double startValue, boolean tuningEnabled, Consumer<Double> onUpdate) {
        m_value = startValue;
        m_name = name;
        m_onUpdate = onUpdate;
        m_tuningEnabled = tuningEnabled;
        onUpdate.accept(m_value);
        if(m_tuningEnabled) {
            SmartDashboard.putNumber(name, m_value);
        }
        AllUpdaters.add(this);
    }

    public double get() {
        return m_value;
    }

    private void checkValue() {
        if(!m_tuningEnabled) {
            return;
        }
        var newValue = SmartDashboard.getNumber(m_name, m_value);
        if(newValue != m_value) {
            m_value = newValue;
            m_onUpdate.accept(m_value);
            ChangedValues.add(m_name);
        }
    }

    public static void checkAll() {
        for (DashboardNumber dashboardNumber : AllUpdaters) {
            dashboardNumber.checkValue();
        }
        SmartDashboard.putStringArray("ChangedValues", ChangedValues.toArray(new String[0]));
    }
}
