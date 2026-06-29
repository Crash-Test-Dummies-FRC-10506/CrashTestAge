package robot.src.main.java.org.frc10506.rebuilt2026.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import robot.src.main.java.org.frc10506.rebuilt2026.commands.DriveCommands.DriveAuto;
import robot.src.main.java.org.frc10506.rebuilt2026.commands.DriveCommands.DriveLooped;
import robot.src.main.java.org.frc10506.rebuilt2026.commands.ShooterCommands.IntaketoShooterCommand;
import robot.src.main.java.org.frc10506.rebuilt2026.commands.ShooterCommands.LowShootRawCommand;
import robot.src.main.java.org.frc10506.rebuilt2026.subsystems.Shooter;
import robot.src.main.java.org.frc10506.rebuilt2026.subsystems.Drivetrain;


public class MainAuto extends SequentialCommandGroup {
  public MainAuto(Drivetrain drivetrain, Shooter shooter) {
    addCommands(
        new DriveAuto(drivetrain, 0, -0.6, 0).withTimeout(0.27), // goes back for 0.3 seconds
        new LowShootRawCommand(shooter), // power up flywheel to shoot
        new WaitCommand(3), // wait 3 secs
        new IntaketoShooterCommand(shooter).withTimeout(10) // shoots the balls with 10 seconds
    );
  }
}