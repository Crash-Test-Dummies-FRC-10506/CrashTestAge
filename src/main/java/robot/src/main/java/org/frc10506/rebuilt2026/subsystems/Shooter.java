package robot.src.main.java.org.frc10506.rebuilt2026.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.src.main.java.org.frc10506.rebuilt2026.util.Constants.ShooterConstants;

import static robot.src.main.java.org.frc10506.rebuilt2026.util.Constants.*;

public class Shooter extends SubsystemBase {
  public SparkMax shooterintake = new SparkMax(10, MotorType.kBrushless);
  public SparkMax shooter = new SparkMax(6, MotorType.kBrushless);;

  private SparkMaxConfig intakeConfig = new SparkMaxConfig();
  private SparkMaxConfig launcherConfig = new SparkMaxConfig();

  private PIDController pidConstants = new PIDController(0.1, 0, 0.0);
  private double KS = 0.0;
  private double KV = 0.0;
    
      public Shooter() {
        intakeConfig.smartCurrentLimit(60).inverted(true);
        shooterintake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        launcherConfig.inverted(true);
        launcherConfig.smartCurrentLimit(60);
        launcherConfig.closedLoop // default values
            .p(ShooterConstants.kP)
            .i(ShooterConstants.kI)
            .d(ShooterConstants.kD)
          .feedForward
            .kV(ShooterConstants.kV)
            .kS(ShooterConstants.kS);
        shooter.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putData("Shooter PID", pidConstants);
        SmartDashboard.putNumber("Shooter Feedforward (kV)", KV);
        SmartDashboard.putNumber("Shooter Feedforward (kS)", KS);
      }

    public void setShooterRaw(double power) {
      this.shooter.set(power);
    }

    public void setShooterVoltage(double voltage) {
      this.shooter.setVoltage(voltage);
    }

    public void setShooterVelocity(double velocity) { // uses pid
      this.shooter.getClosedLoopController().setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1); // slot 1 is velocity control
    }

    public void setShooterIntake(double power) {
        this.shooterintake.set(power);
    }

  @Override
  public void periodic() {  // TODO: clean this up a bit with the constants stuff
    ShooterConstants.closevoltage = SmartDashboard.getNumber("Close Voltage (Near Hub)", 0);
    ShooterConstants.farvoltage = SmartDashboard.getNumber("Far Voltage (Tower)", 0);
    ShooterConstants.closevelocity = SmartDashboard.getNumber("Close Velocity (Near Hub)", 0);
    ShooterConstants.farvelocity = SmartDashboard.getNumber("Far Velocity (Tower)", 0);

    launcherConfig.closedLoop.p(pidConstants.getP()).i(pidConstants.getI()).d(pidConstants.getD()).feedForward.kS(KS).kV(KV);
    shooter.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
    //SmartDashboard.putNumber("Shooter Close Voltage", ShooterConstants.closevoltage);
    //SmartDashboard.putNumber("Shooter Far Voltage", ShooterConstants.farvoltage);
    //SmartDashboard.putNumber("Shooter Close Velocity", ShooterConstants.closevelocity);
    //SmartDashboard.putNumber("Shooter Far Velocity", ShooterConstants.farvelocity);
  }
}