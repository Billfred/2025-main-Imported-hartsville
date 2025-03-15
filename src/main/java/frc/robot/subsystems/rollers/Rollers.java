package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
    private final PWMMotorController upperRoller = new PWMSparkMax(RollerConstants.kUpperId);
    private final PWMMotorController lowerRoller = new PWMSparkMax(RollerConstants.kLowerId);

    public Rollers() {
        configureMotors();
    }

    private void configureMotors() {
        upperRoller.addFollower(follower); // Breaking the correlation would let us do algae removal
        upperRoller.setInverted(RollerConstants.invert);
        lowerRoller.setInverted(RollerConstants.invertFollower);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(getName() + "/Upper/Output", upperRoller.get());
        SmartDashboard.putNumber(getName() + "/Upper/AppliedVoltage", upperRoller.getVoltage());
        SmartDashboard.putNumber(getName() + "/Lower/Output", lowerRoller.get());
        SmartDashboard.putNumber(getName() + "/Lower/AppliedVoltage", lowerRoller.getVoltage());
    }    

    private void setVolts(double volts) {
        upperRoller.setVoltage(volts);
    }

    public Command setVoltsCommand(double volts) {
        return run(() -> setVolts(volts));
    }
}