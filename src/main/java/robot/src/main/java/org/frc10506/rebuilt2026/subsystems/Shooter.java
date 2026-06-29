package robot.src.main.java.org.frc10506.rebuilt2026.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.src.main.java.org.frc10506.rebuilt2026.util.Constants.ShooterConstants;

import static robot.src.main.java.org.frc10506.rebuilt2026.util.Constants.*;

public class Shooter extends SubsystemBase {
  private final SparkMax shooterintake;
  private final  SparkMax shooter;

  private PIDController pidConstants = new PIDController(0.1, 0, 0.0);
    
      public Shooter() {
        shooter = new SparkMax(6, MotorType.kBrushless);
        shooterintake = new SparkMax(10, MotorType.kBrushless);

        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(60).inverted(true);
        shooterintake.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SparkMaxConfig launcherConfig = new SparkMaxConfig();
        launcherConfig.inverted(true);
        launcherConfig.smartCurrentLimit(60);
        launcherConfig.closedLoop
            .p(pidConstants.getP())
            .i(pidConstants.getI())
            .d(pidConstants.getD())
          .feedForward
            .kV(ShooterConstants.kV)
            .kS(ShooterConstants.kS);
        shooter.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }

    public void setShooterRaw(double power) {
      this.shooter.set(power);
    }

    public void setShooterVoltage(double voltage) {
      this.shooter.setVoltage(voltage);
    }

    public void setShooterVelocity(double velocity) { // uses pid
      this.shooter.getClosedLoopController().setSetpoint(velocity, ControlType.kVelocity);
    }

    public void setShooterIntake(double power) {
        this.shooterintake.set(power);
    }

  @Override
  public void periodic() { // TODO: clean this up a bit with the constants stuff
    SmartDashboard.putNumber("Shooter Close Voltage", ShooterConstants.closevoltage);
    SmartDashboard.putNumber("Shooter Far Voltage", ShooterConstants.farvoltage);

    SmartDashboard.putData("Shooter PID", pidConstants);
    SmartDashboard.putNumber("Shooter Feedforward (kV)", ShooterConstants.kV);
    SmartDashboard.putNumber("Shooter Feedforward (kS)", ShooterConstants.kS);
    SmartDashboard.putNumber("Shooter Close Velocity", ShooterConstants.closevelocity);
    SmartDashboard.putNumber("Shooter Far Velocity", ShooterConstants.farvelocity);
  }
}