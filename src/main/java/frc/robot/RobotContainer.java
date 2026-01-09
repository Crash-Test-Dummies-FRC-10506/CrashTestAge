// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class RobotContainer {
  public final DriveSubsystem m_drive = new DriveSubsystem();
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureAutos();
  }

  private void configureAutos() {
    m_autoChooser.addOption("Main Auto", Autos.mainAuto(this));
    m_autoChooser.addOption("Untested Auto", Autos.untestedAuto(this));
    SmartDashboard.putData("Auto Select", m_autoChooser);
  }

  public Command getAuto() {
    return m_autoChooser.getSelected();
  }

  public void teleop() {
    double drivepower = 0.668;

    if (m_driverController.rightBumper().getAsBoolean()) { // brake control
      drivepower = 0.3;
    }

    SmartDashboard.putNumber("Drive Speed", drivepower); // just in case if we need to change it
    m_drive.driveCartesian(-m_driverController.getLeftY() * drivepower, m_driverController.getLeftX() * drivepower, m_driverController.getRightX() * drivepower);
  }

  public void simulationDrive() {
    m_drive.driveCartesianFieldRelative(-m_driverController.getLeftY(), -m_driverController.getLeftX(), -m_driverController.getRightX());
  }

  public void reset() {
    m_drive.reset();
  }
}
