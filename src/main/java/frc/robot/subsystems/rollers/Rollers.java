package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
    private final PWMMotorController leader = new PWMSparkMax(RollerConstants.kLeaderId);
    private final PWMMotorController follower = new PWMSparkMax(RollerConstants.kFollowerId);

    public Rollers() {
        configureMotors();
    }

    private void configureMotors() {
        leader.addFollower(follower);
        leader.setInverted(RollerConstants.invert);
        follower.setInverted(RollerConstants.invertFollower);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(getName() + "/Leader/Output", leader.get());
        SmartDashboard.putNumber(getName() + "/Leader/AppliedVoltage", leader.getVoltage());
        SmartDashboard.putNumber(getName() + "/Follower/Output", follower.get());
        SmartDashboard.putNumber(getName() + "/Follower/AppliedVoltage", follower.getVoltage());
    }    

    private void setVolts(double volts) {
        leader.setVoltage(volts);
    }

    public Command setVoltsCommand(double volts) {
        return run(() -> setVolts(volts));
    }
}