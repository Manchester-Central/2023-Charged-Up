// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer.ArmMode;
import frc.robot.subsystems.ArduinoIO;

public class DefaultLedCommand extends CommandBase {
  private ArduinoIO m_ledStrip;
  private Supplier<ArmMode> m_armModeSupplier;

  public DefaultLedCommand(ArduinoIO leds, Supplier<ArmMode> armModeSupplier) {
    m_ledStrip = leds;
    m_armModeSupplier = armModeSupplier;
    addRequirements(m_ledStrip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.isDSAttached() == false) {
      m_ledStrip.setRGB(200, 200, 200);
      return;
    } 
    
    // Check if we are in autonomous OR if we are disabled. If so, we will show our current alliance color.
    if(DriverStation.isAutonomous() || DriverStation.isDisabled()) {
      if(DriverStation.getAlliance() == Alliance.Red) {
        m_ledStrip.setRGB(255, 0, 0);
      } else if(DriverStation.getAlliance() == Alliance.Blue) {
        m_ledStrip.setRGB(0, 0, 255);
      }
      return;
    }

    // Check whether we are in cubee mode or cone mode. Set the led color appropriately.
    if(m_armModeSupplier.get() == ArmMode.Cone) {
      m_ledStrip.setRGB(255, 100, 0);
    } else if(m_armModeSupplier.get() == ArmMode.Cube) {
      m_ledStrip.setRGB(115, 0, 75);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
