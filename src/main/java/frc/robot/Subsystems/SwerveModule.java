// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {
  TalonFX azimuth;
  TalonFX drive;
  CANcoder cancoder;
  double cancoderOffset;

  PositionVoltage azimuthControl = new PositionVoltage(0.0);
  VoltageOut driveOpenLoop = new VoltageOut(0.0);
  VelocityVoltage driveClosedLoop = new VelocityVoltage(0.0);

  StatusSignal<Double> cancoderAngle = cancoder.getAbsolutePosition();
  StatusSignal<Double> azimuthAngle = azimuth.getPosition();
  StatusSignal<Double> driveSpeed = drive.getVelocity();
  StatusSignal<Double> drivePosition = drive.getPosition();

  public SwerveModule(int azimuthID, int driveID, int cancoderID, double cancoderOffset) {
    azimuth = new TalonFX(azimuthID);
    drive = new TalonFX(driveID);
    cancoder = new CANcoder(cancoderID);
    this.cancoderOffset = cancoderOffset;

    resetToAbsolute();
  }

  public SwerveModule(SwerveModuleConstants constants) {
    this(constants.azimuthID, constants.driveID, constants.encoderID, constants.azimuthOffset);
  }

  public void setTargetState(SwerveModuleState state, boolean isOpenLoop) {
    state = SwerveModuleState.optimize(state, getIntegratedAzimuth());
    setTargetAzimuth(state.angle);
    if (isOpenLoop) {
      // In this case the units in SwerveModuleState don't match the units we are using
      setDriveVolts(state.speedMetersPerSecond);
    } else {
      setDriveVelocity(state.speedMetersPerSecond);
    }
  }

  public void setTargetAzimuth(Rotation2d angle) {
    azimuth.setControl(azimuthControl.withPosition(angle.getRotations() * Constants.azimuthRatio));
  }

  public void setDriveVolts(double volts) {
    drive.setControl(driveOpenLoop.withOutput(volts));
  }

  public void setDriveVelocity(double metersPerSecond) {
    double rotationsPerSecond =
        (metersPerSecond * Constants.driveRatio) / (Constants.wheelDiameter * Math.PI);
    drive.setControl(
        driveClosedLoop
            .withVelocity(rotationsPerSecond)
            .withFeedForward(Constants.driveFeedforward.calculate(rotationsPerSecond)));
  }

  /** Uses the integrated falcon encoder */
  public Rotation2d getIntegratedAzimuth() {
    return Rotation2d.fromRotations(azimuthAngle.getValue() / Constants.azimuthRatio);
  }

  /** Uses the CANCoder with the offset */
  public Rotation2d getAbsoluteAzimuth() {
    return Rotation2d.fromRotations(cancoderAngle.getValue() + cancoderOffset);
  }

  public double getSpeedMetersPerSecond() {
    return (driveSpeed.getValue() * Constants.wheelDiameter * Math.PI) / Constants.driveRatio;
  }

  public double getPositionMeters() {
    return (drivePosition.getValue() * Constants.wheelDiameter * Math.PI) / Constants.driveRatio;
  }

  public void resetToAbsolute() {
    azimuth.setRotorPosition(cancoderAngle.getValue() + cancoderOffset);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getIntegratedAzimuth());
  }
}
