// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcoder.CANandcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {
  TalonFX pivot = new TalonFX(Constants.pivotID);
  CANandcoder encoder = new CANandcoder(Constants.pivotEncoderID);

  PositionVoltage positionControl = new PositionVoltage(0.0);

  StatusSignal<Double> pivotError = pivot.getClosedLoopError();

  public PivotSubsystem() {}

  /** Command that goes to the specified target */
  public CommandBase runToRotation(Rotation2d rotation) {
    return this.run(
            () ->
                pivot.setControl(
                    positionControl
                        .withPosition(pivotToMotorAngle(rotation.getRotations()))
                        // This feedforward only helps at steady state. Add a motion profile for smoother motion.
                        .withFeedForward(Constants.pivotFeedforward.calculate(rotation.getRadians(), 0.0))));
  }

  /** Command that goes to the specified target and ends when setpoint is reached */
  public CommandBase runUntilRotation(Rotation2d rotation) {
    return runToRotation(rotation).until(() -> isAtSetpoint());
  }

  public boolean isAtSetpoint() {
    return pivotToMotorAngle(pivotError.getValue()) < Constants.pivotAllowableError;
  }

  private static double pivotToMotorAngle(double rotations) {
    return rotations / Constants.pivotRatio;
  }

  @Override
  public void periodic() {}
}
