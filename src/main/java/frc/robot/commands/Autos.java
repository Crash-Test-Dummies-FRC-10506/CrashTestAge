// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
//import frc.robot.subsystems.commands.DriveCommand;
import frc.robot.subsystems.commands.DriveToPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command mainAuto(RobotContainer bot) {
    return Commands.sequence(
      new DriveToPoint(bot.m_drive, new Pose2d(14.796, 4.033, new Rotation2d(0)), Units.inchesToMeters(1), 0.45)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
