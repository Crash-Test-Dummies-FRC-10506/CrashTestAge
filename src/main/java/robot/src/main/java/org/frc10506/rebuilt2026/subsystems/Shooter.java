package robot.src.main.java.org.frc10506.rebuilt2026.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.src.main.java.org.frc10506.rebuilt2026.util.Constants.ShooterConstants;

import static robot.src.main.java.org.frc10506.rebuilt2026.util.Constants.*;

public class Shooter extends SubsystemBase {
  public SparkMax shooterintake = new SparkMax(10, MotorType.kBrushless);
  public SparkMax shooter = new SparkMax(6, MotorType.kBrushless);;

  private SparkMaxConfig intakeConfig = new SparkMaxConfig();
  private SparkMaxConfig launcherConfig = new SparkMaxConfig();
  private double mode = 0;

  private PIDController pidConstants = new PIDController(0.01, 0, 0.0);
  private double KS = 0.1;
  private double KV = 0.001;
    
      public Shooter() {
        intakeConfig.smartCurrentLimit(60).inverted(true);
        shooterintake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        launcherConfig.inverted(true);
        launcherConfig.smartCurrentLimit(60);
        launcherConfig.voltageCompensation(10);
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
        SmartDashboard.putNumber("Shooter Mode:", mode);
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

    public Command setMode(double modetype) {
        return runOnce(() -> {
            mode = modetype;
        });
    }

  @Override
  public void periodic() {  // TODO: clean this up a bit with the constants stuff and use finite state machine for modes
    ShooterConstants.closevelocity = SmartDashboard.getNumber("Close Velocity (Near Hub)", 0);
    ShooterConstants.farvelocity = SmartDashboard.getNumber("Far Velocity (Tower)", 0);

    SmartDashboard.putNumber("Current Shooter Velocity", shooter.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter NEO Power", shooter.get());

    launcherConfig.closedLoop.p(pidConstants.getP()).i(pidConstants.getI()).d(pidConstants.getD()).feedForward.kS(KS).kV(KV);
    shooter.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (mode == 0) {
      setShooterVelocity(0);
    } else if (mode == 1) {
      setShooterVelocity(ShooterConstants.closevelocity);
    } else if (mode == 2) {
      setShooterVelocity(ShooterConstants.farvelocity);
    } else {
      setShooterVelocity(0);
    }
  }

  /*@Override
  public void simulationPeriodic() {
    ShooterConstants.closevelocity = SmartDashboard.getNumber("Close Velocity (Near Hub)", 0);
    ShooterConstants.farvelocity = SmartDashboard.getNumber("Far Velocity (Tower)", 0);

    launcherConfig.closedLoop.p(pidConstants.getP()).i(pidConstants.getI()).d(pidConstants.getD()).feedForward.kS(KS).kV(KV);
    shooter.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }*/
}
