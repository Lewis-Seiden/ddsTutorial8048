// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  TalonFX intake = new TalonFX(Constants.intakeID);

  VoltageOut voltageControl = new VoltageOut(0.0);

  public IntakeSubsystem() {}

  public CommandBase run(double voltage) {
    return this.run(() -> intake.setControl(voltageControl.withOutput(voltage)));
  }

  public CommandBase intake() {
    return this.run(10.0);
  }

  public CommandBase outtake() {
    return this.run(-10.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
