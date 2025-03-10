package frc.robot.subsystems.pivot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SparkUtil;

public class Pivot extends SubsystemBase {
    private final SparkMax leader = new SparkMax(PivotConstants.kLeaderId, MotorType.kBrushless);
    private final SparkMax follower = new SparkMax(PivotConstants.kFollowerId, MotorType.kBrushless);
    private final SparkClosedLoopController controller = leader.getClosedLoopController();

    private final RelativeEncoder leaderEncoder = leader.getEncoder();
    private final RelativeEncoder followerEncoder = follower.getEncoder();

    public Pivot() {
        configureMotors();
    }

    private void configureMotors() {
        SparkBaseConfig config = new SparkMaxConfig();

        config
            .idleMode(PivotConstants.kIdleMode)
            .voltageCompensation(12.0)
            .smartCurrentLimit(PivotConstants.kCurrentLimit)
            .inverted(PivotConstants.kInvert);

        config.encoder
            .positionConversionFactor((2 * Math.PI) / PivotConstants.kReduction)
            .velocityConversionFactor(((2 * Math.PI) / 60.0) / PivotConstants.kReduction)
            .uvwMeasurementPeriod(20)
            .uvwAverageDepth(2);

        SparkUtil.tryUntilOk(
            leader,
                5,
                () -> leader.configure(
                        config,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));

        SparkUtil.tryUntilOk(
            leader,
            5,
            () -> leaderEncoder.setPosition(0));
                

        // Modify the config to have follower specific stuff before we apply
        config.follow(leader, PivotConstants.kInvertFollower);
        SparkUtil.tryUntilOk(
            follower,
                5,
                () -> follower.configure(
                            config,
                            ResetMode.kResetSafeParameters,
                            PersistMode.kPersistParameters));
    
        SparkUtil.tryUntilOk(
            follower,
            5,
            () -> followerEncoder.setPosition(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(getName() + "/PositionRad", leaderEncoder.getPosition());
        SmartDashboard.putNumber(getName() + "/VelocityRadPerSec", leaderEncoder.getVelocity());
        SmartDashboard.putNumber(getName() + "/AppliedVoltage", leader.getBusVoltage() * leader.getAppliedOutput());
        SmartDashboard.putNumber(getName() + "/OutputCurrent", leader.getOutputCurrent());
        SmartDashboard.putNumber(getName() + "/TemperatureCelsius", leader.getMotorTemperature());
    }

    private void setVolts(double volts) {
        leader.setVoltage(volts);
    }

    public Command setVoltsCommand(double volts) {
        return runOnce(() -> setVolts(volts));
    }

    private void setGoal(Rotation2d angle) {
      controller.setReference(
            angle.getRadians(),
            ControlType.kMAXMotionPositionControl);
    }

    public Command setGoalCommand(Rotation2d angle) {
        return runOnce(() -> setGoal(angle));
    }
}
