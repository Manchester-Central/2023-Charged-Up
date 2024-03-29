// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;
import frc.robot.Constants.DebugConstants;

/** Manages the logs recorded into the .csv file, stored on a flash drive on the robot*/
public class LogManager {
    private ShuffleboardTab m_shuffleboardTab;
    private List<String> m_headers = new ArrayList<String>();
    private Map<String, Supplier<String>> m_suppliers = new HashMap<String, Supplier<String>>();
    private LoggingThread m_loggingThread;

    public LogManager() {
        m_loggingThread = new LoggingThread();
        m_shuffleboardTab = Shuffleboard.getTab("Logging");
        addNumber("timeMs", DebugConstants.IsDebugMode, () -> Robot.getCurrentTimeMs());
    }

    public void addBoolean(String title, boolean willLogShuffleboard, BooleanSupplier valueSupplier) {
        m_headers.add(title);
        m_suppliers.put(title, () -> String.valueOf(valueSupplier.getAsBoolean()));
        if (willLogShuffleboard) {
            m_shuffleboardTab.addBoolean(title, valueSupplier);
        }
    }

    public void addNumber(String title, boolean willLogShuffleboard, DoubleSupplier valueSupplier) {
        m_headers.add(title);
        m_suppliers.put(title, () -> String.valueOf(valueSupplier.getAsDouble()));
        if (willLogShuffleboard) {
            m_shuffleboardTab.addNumber(title, valueSupplier);
        }
    }

    public void addString(String title, boolean willLogShuffleboard, Supplier<String> valueSupplier) {
        m_headers.add(title);
        m_suppliers.put(title, valueSupplier);
        if (willLogShuffleboard) {
            m_shuffleboardTab.addString(title, valueSupplier);
        }
    }

    public void writeHeaders() {
        StringBuilder sb = new StringBuilder();
        for (String header : m_headers) {
            sb.append(header + ",");
        }
        m_loggingThread.enqueue(sb.toString());
    }

    public void update() {
        try {
            if (m_loggingThread.getState() == Thread.State.NEW) {
                // Robot's unix time might be 1970, start logging AFTER the driver station is attached to avoid that. :)))  
                if (DriverStation.isDSAttached()) {
                    m_loggingThread.start(); 
                }
            } else {
                StringBuilder sb = new StringBuilder();
                for (String header : m_headers) {
                    sb.append(m_suppliers.get(header).get() + ",");
                }
                m_loggingThread.enqueue(sb.toString());
            }
        } catch (Exception ex) {
            // If logging fails, print the stack trace, but don't rethrow it
            ex.printStackTrace();
        }
    }
}
