// sorry but this code is a bit messier so will get back to this soon

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterTest extends SubsystemBase {

    SparkMax m_shooterMotor = new SparkMax(ShooterConstants.kShooterMotorID, MotorType.kBrushed);
    private SparkMaxConfig m_config = new SparkMaxConfig();

    private TrapezoidProfile m_profile = new TrapezoidProfile(new Constraints(ShooterConstants.kMaxVelocity, ShooterConstants.kMaxAccelrate));
    private TrapezoidProfile.State m_targetPower = new State();
    
    private PIDController pidConstants = new PIDController(ShooterConstants.kPID_Proportional, 0.0, 0.0);

    private double m_motorPower = 0;
    private double m_shooterAngle = 5; // Degrees

    public ShooterTest() {
        m_config.closedLoop.pid(ShooterConstants.kPID_Proportional, 0.0, 0.0);
        m_config.inverted(true); //
        m_config.idleMode(IdleMode.kBrake);
        m_config.smartCurrentLimit(90);

        m_shooterMotor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        SmartDashboard.putData("Shooter PID", pidConstants);
    }

    // Called periodically
    // Returns false if not at target power yet and true if up to speed and ready to shoot
   // public boolean shoot() {
    //    if (Math.abs(m_targetPower - getActualPower()) < Constants.ShooterConstants.kMaxPowerError) {
    //        return true;
    //    }
    //}

    // Implementation depends on final shooter CAD
   // public void aimToTarget(Pose2D robotPos, Pose2D targetPos) {
        // Calculate trajectory given targetPos (account for where the tip of the shooter is, given robotPos)
        // Set necessary power and shooter angle (unless shooter angle is constant in the CAD)

    //    double trgAng; // Target angle // Respective of kMaxAngle // DEPENDS ON FINAL SHOOTER CAD
   //     double linVel;
    //    double angVel;
    //    double pwr; // Respective of kMaxPower and set voltage; calculated using trajectory math stuff idk
    //    setPower(pwr);
    //    setAngle(trgAng);
    //}

    public void setReference(double val, ControlType type) {
        m_shooterMotor.getClosedLoopController().setReference(val, type, ClosedLoopSlot.kSlot0, 0.0);
    }

    @Override
    public void periodic() {
       // setReference(m_setpower.position, ControlType.kPosition);
        SmartDashboard.putNumber("NEO Power", m_shooterMotor.get());
        
        m_config.closedLoop.pid(pidConstants.getP(), pidConstants.getI(), pidConstants.getD());
        m_shooterMotor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Implementation depends on final shooter CAD
    public void setAngle(double angle) {
        m_shooterAngle = angle;
    }
    public void setPower(double pwr) {
        m_targetPower.position = pwr;
        m_shooterMotor.set(pwr);
    }
    public double getActualPower() {
        double appliedOutput  = m_shooterMotor.getAppliedOutput();
        double busVoltage     = m_shooterMotor.getBusVoltage();
        double outputCurrent  = m_shooterMotor.getOutputCurrent();
        double appliedVoltage = appliedOutput * busVoltage;

        double actualPower = appliedVoltage * outputCurrent;
        return actualPower;
    }
}
    