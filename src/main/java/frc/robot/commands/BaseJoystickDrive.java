// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.DashboardNumber;

public abstract class BaseJoystickDrive extends CommandBase {

  protected interface JoystickFilter {
    double get();
  }

  protected class JoystickSlewer implements JoystickFilter {
    SlewRateLimiter m_slewRateLimiter;
    Supplier<Double> m_joystickSupplier;

    JoystickSlewer(Supplier<Double> joystickSupplier) {
      new DashboardNumber("Driver/SlewRateLimit", 0.2, (newLimit) -> {
        m_slewRateLimiter = new SlewRateLimiter(newLimit);
      });
      m_joystickSupplier = joystickSupplier;
    }

    public double get() {
      return m_slewRateLimiter.calculate(m_joystickSupplier.get());
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
    m_slewedLeftX = new JoystickSlewer(() -> driverController.getLeftX());
    m_slewedLeftY = new JoystickSlewer(() -> driverController.getLeftY());
    m_slewedRightX = new JoystickSlewer(() -> driverController.getRightX());
    m_slewedRightY = new JoystickSlewer(() -> driverController.getRightY());

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
