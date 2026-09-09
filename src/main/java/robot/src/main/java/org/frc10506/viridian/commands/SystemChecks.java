package robot.src.main.java.org.frc10506.viridian.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import robot.src.main.java.org.frc10506.viridian.subsystems.Shooter;
import robot.src.main.java.org.frc10506.viridian.subsystems.MechanumDrive;

public class SystemChecks {
    private final Shooter shooter;
    private final MechanumDrive drive;

    public SystemChecks(Shooter shooter, MechanumDrive drive) {
        this.shooter = shooter;
        this.drive = drive;

        SmartDashboard.putData("System Checks/Shooter", shooter(shooter));
        SmartDashboard.putData("System Checks/Intake", intake(shooter));
        SmartDashboard.putData("System Checks/Drive", drive(drive));
    }

    public Command shooter(Shooter shooter) {
        return new ShooterCommand(shooter, 1).withTimeout(1.5);
    }

    public Command drive(MechanumDrive drive) {
        return new DriveAuto(drive, 0, -0.5, 0).withTimeout(1.5);
    }

    public Command intake(Shooter shooter) {
        return new ShooterFeederCommand(shooter, 1).withTimeout(1.5);
    }
}