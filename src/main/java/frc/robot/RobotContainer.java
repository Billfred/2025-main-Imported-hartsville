// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ControllerConstants;
import frc.robot.controllers.CommandCustomXboxController;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.rollers.RollerConstants;
import frc.robot.subsystems.rollers.Rollers;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandCustomXboxController driverController = new CommandCustomXboxController(
            ControllerConstants.kDriverControllerPort);

    private final CommandCustomXboxController operatorController = new CommandCustomXboxController(
        ControllerConstants.kOperatorControllerPort);    

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final Pivot pivot = new Pivot();
    private final Rollers rollers = new Rollers();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        driveSubsystem.setDefaultCommand(
                driveSubsystem.driveClosedLoopCommand(
                        // Negate because on controllers up is negative; up should be positive
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getRightX()));
        
        // Pivot
        
        operatorController.povUp() //Turbo arm up
            .and(operatorController.leftBumper())
            .onTrue(pivot.setVoltsCommand(PivotConstants.kTurboVoltage))
            .onFalse(pivot.setVoltsCommand(0));

        operatorController.povUp() //Standard arm up
            .and(operatorController.leftBumper().negate()) //The negate flips what it's looking for. Thanks for the protip, Allen!
            .onTrue(pivot.setVoltsCommand(PivotConstants.kMoveVoltage))
            .onFalse(pivot.setVoltsCommand(0));

        operatorController.povDown() //Turbo arm down
            .and(operatorController.leftBumper())
            .onTrue(pivot.setVoltsCommand(-PivotConstants.kTurboVoltage))
            .onFalse(pivot.setVoltsCommand(0));

        operatorController.povDown() //Standard arm down
            .and(operatorController.leftBumper().negate())
            .onTrue(pivot.setVoltsCommand(-PivotConstants.kMoveVoltage))
            .onFalse(pivot.setVoltsCommand(0));
        
        // Rollers
        operatorController.leftTrigger()
            .whileTrue(rollers.setVoltsCommand(-RollerConstants.kIntakeVoltage))
            .onFalse(rollers.setVoltsCommand(0));

        operatorController.rightTrigger()
            .whileTrue(rollers.setVoltsCommand(RollerConstants.kShootVoltage))
            .onFalse(rollers.setVoltsCommand(0));

        operatorController.a()
            .whileTrue(rollers.setVoltsCommand(RollerConstants.kShootVoltage))
            .onFalse(rollers.setVoltsCommand(0));
    }

   public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            driveSubsystem.driveClosedLoopCommand(() -> 0.9, () -> 0)//.repeatedly()
                .withTimeout(1)
                .andThen(driveSubsystem.driveClosedLoopCommand(() -> 0, () -> 0))
        );
        //    return Commands.sequence(
        //        Commands.deadline(
        //        Commands.waitSeconds(2),
        //        driveSubsystem.driveClosedLoopCommand(() -> 0.3, () -> 0.0)),
        //        Commands.deadline(Commands.waitSeconds(2), 
        //        driveSubsystem.driveClosedLoopCommand(() -> 0.0, () -> 0.0)));
   }
}
