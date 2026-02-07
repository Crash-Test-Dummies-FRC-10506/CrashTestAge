// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.MechanumDrive;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.commands.PowerIntake;
import frc.robot.subsystems.commands.PowerShooter;
import frc.robot.subsystems.commands.PowerShooterMotionMagic;


public class RobotContainer {
  public final MechanumDrive m_drive = new MechanumDrive();
  public final Intake m_intake = new Intake();
  public final Shooter m_shooter = new Shooter();
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_shooterController =
      new CommandXboxController(OperatorConstants.kShooterControllerPort);

  public RobotContainer() {
    configureAutos();
    configureBindings();
  }

  private void configureBindings() {
    m_shooterController.a().onTrue(new PowerIntake(m_intake, 1));
    m_shooterController.b().onTrue(new PowerShooter(m_shooter, 0.9));
    m_shooterController.y().onTrue(new PowerShooterMotionMagic(m_shooter, 75));
  }

  private void configureAutos() {
    m_autoChooser.addOption("Main Auto", Autos.mainAuto(this));
    SmartDashboard.putData("Auto Select", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public void teleop() {
    double drivepower = 0.668;

    if (m_driverController.rightBumper().getAsBoolean()) { // brake control
      drivepower = 0.3;
    }
    if (m_driverController.leftBumper().getAsBoolean()) { // speed up for the trenches
      drivepower = 0.9;
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
