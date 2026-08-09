package robot.src.main.java.org.frc10506.viridian.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;

import framework.src.main.java.org.frc10506.framework.scheduler.subsystem.TickedSubsystem;
import robot.src.main.java.org.frc10506.viridian.subsystems.MechanumDrive;
import static robot.src.main.java.org.frc10506.viridian.util.IDs.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Shooter implements TickedSubsystem {

    private final SparkMax shooter;
    private final SparkMax feeder;

    public Shooter() {
        this.shooter = new SparkMax(SHOOTER, MotorType.kBrushless);
        this.feeder = new SparkMax(FEEDER, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        this.shooter.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }

    public void setShooterSpeed(double speed) {
        this.shooter.set(speed);
    }

    public void setShooterVoltage(double voltage) {
        this.shooter.setVoltage(voltage);
    }

    public void setShooterRPM(double rpm) {
        this.shooter.setVoltage((rpm / 5500) * 12);
    }

    public void setFeederSpeed(double speed) {
        this.feeder.set(speed);
    }

    @Override
    public void periodic() {
    SmartDashboard.putNumber("Shooter NEO Power", shooter.get());
    }

}
