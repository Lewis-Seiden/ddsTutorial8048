// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.PivotSubsystem;
import frc.robot.Subsystems.SwerveModule;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {
  SwerveSubsystem swerve =
      new SwerveSubsystem(
          new SwerveModule(Constants.module0),
          new SwerveModule(Constants.module1),
          new SwerveModule(Constants.module2),
          new SwerveModule(Constants.module3));
  PivotSubsystem pivot = new PivotSubsystem();
  IntakeSubsystem intake = new IntakeSubsystem();

  CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        swerve.drive(
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> controller.getRightX(),
            true,
            true));
    pivot.setDefaultCommand(pivot.runToRotation(Rotation2d.fromDegrees(90)));
    // Run at low power to hold cubes in better
    intake.setDefaultCommand(intake.run(0.5));

    controller
        .leftTrigger()
        .whileTrue(pivot.runToRotation(Rotation2d.fromDegrees(0)).alongWith(intake.intake()));
    controller
        .rightTrigger()
        .whileTrue(pivot.runUntilRotation(Rotation2d.fromDegrees(45)).andThen(intake.outtake()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
