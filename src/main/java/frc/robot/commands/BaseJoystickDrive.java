// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public abstract class BaseJoystickDrive extends CommandBase {

  public static boolean UseSlewer = true;

  protected interface JoystickFilter {
    double get();
  }

  protected class JoystickSlewer implements JoystickFilter {
    SlewRateLimiter m_slewRateLimiter = new SlewRateLimiter(0.2); // Test for a good value. This is amount of input change in one second, so should allow 0 to full in 1/6 a second
    Supplier<Double> m_joystickSupplier;
    
    JoystickSlewer(Supplier<Double> joystickSupplier) {
      m_joystickSupplier = joystickSupplier;
    }

    public double get() {
      return m_slewRateLimiter.calculate(m_joystickSupplier.get());
    }
  }

  protected class JoystickLowPassFilter implements JoystickFilter {
    double lastFilteredValue = 0;
    final double FilteringConstant = 0.2;
    Supplier<Double> m_joystickSupplier;

    JoystickLowPassFilter(Supplier<Double> joystickSupplier) {
      m_joystickSupplier = joystickSupplier;
    }

    /**
     * Runs a low pass filter on the supplied joystick data
     * From Dan:
     * X = Y + (Y-Z) A
     * Correct formula: X = Y + (Z-Y) A (Josh swapped Z and Y)
     * X: new filtered value
     * Y: old filtered value
     * Z: new unfiltered value
     * A: filtering Constant
     */
    public double get() {
      double newValue = m_joystickSupplier.get();
      double newFilteredValue = lastFilteredValue + ((newValue - lastFilteredValue) * FilteringConstant);
      lastFilteredValue = newFilteredValue;
      return newFilteredValue;
    }
  }

  protected final SwerveDrive m_swerveDrive;
  protected final Gamepad m_driverController;
  protected final JoystickFilter m_slewedLeftX;
  protected final JoystickFilter m_slewedLeftY;
  protected final JoystickFilter m_slewedRightX;
  protected final JoystickFilter m_slewedRightY;

  /** Shares some code for all joystick drives */
  public BaseJoystickDrive(SwerveDrive swerveDrive, Gamepad driverController) {
    m_swerveDrive = swerveDrive;
    m_driverController = driverController;
    if (UseSlewer) {
      m_slewedLeftX = new JoystickSlewer(() -> driverController.getLeftX());
      m_slewedLeftY = new JoystickSlewer(() -> driverController.getLeftY());
      m_slewedRightX = new JoystickSlewer(() -> driverController.getRightX());
      m_slewedRightY = new JoystickSlewer(() -> driverController.getRightY());
    } else {
      m_slewedLeftX = new JoystickLowPassFilter(() -> driverController.getLeftX());
      m_slewedLeftY = new JoystickLowPassFilter(() -> driverController.getLeftY());
      m_slewedRightX = new JoystickLowPassFilter(() -> driverController.getRightX());
      m_slewedRightY = new JoystickLowPassFilter(() -> driverController.getRightY());
    }


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
