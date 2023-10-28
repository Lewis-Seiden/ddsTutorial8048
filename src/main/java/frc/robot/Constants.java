// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static class SwerveModuleConstants {
    public final int azimuthID;
    public final int driveID;
    public final double azimuthOffset;
    public final int encoderID;

    public SwerveModuleConstants(int azimuthID, int driveID, int encoderID, double azimuthOffset) {
      this.azimuthID = azimuthID;
      this.driveID = driveID;
      this.encoderID = encoderID;
      this.azimuthOffset = azimuthOffset;
    }
  }

  public static final double loopTimeSeconds = 0.02;

  // Swerve Constants

  public static final double azimuthRatio = 150.0 / 7.0; // SDS Mk4i
  public static final double driveRatio = 6.75 / 1.0; // SDS Mk4i L2
  public static final double maxModuleSpeed = Units.feetToMeters(16.0); // SDS Mk4i L2 free speed
  public static final double wheelDiameter = Units.inchesToMeters(4.0);
  // Tuning values taken from 364lib
  public static final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(0.32, 1.51);

  // Chassis size
  public static final double trackWidth = Units.inchesToMeters(22.7);
  public static final double wheelBase = Units.inchesToMeters(24.5);

  public static final int gyroID = 0;

  public static final SwerveModuleConstants module0 = new SwerveModuleConstants(0, 1, 0, 0.0);
  public static final SwerveModuleConstants module1 = new SwerveModuleConstants(2, 3, 1, 0.0);
  public static final SwerveModuleConstants module2 = new SwerveModuleConstants(4, 5, 2, 0.0);
  public static final SwerveModuleConstants module3 = new SwerveModuleConstants(6, 7, 3, 0.0);

  public static final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  static {
    driveConfig.Slot0.kP = 0.12;

    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    driveConfig.CurrentLimits.SupplyCurrentLimit = 35;
    driveConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    driveConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // 364lib uses a ramp to help with tipping, but this bot is so short it probably doesnt matter
    driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.15;
  }

  public static final TalonFXConfiguration azimuthConfig = new TalonFXConfiguration();
  static {
    azimuthConfig.Slot0.kP = 4.8;
    azimuthConfig.Slot0.kD = 0.002;

    azimuthConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    azimuthConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    azimuthConfig.CurrentLimits.SupplyCurrentLimit = 25;
    azimuthConfig.CurrentLimits.SupplyCurrentThreshold = 40;
    azimuthConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    azimuthConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  }

  public static final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
  static {
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
  }

  // Pivot Constants

  public static final int pivotID = 10;
  public static final int pivotEncoderID = 0;

  public static final double pivotRatio = 1.0 / 1.0;
  public static final double pivotAllowableError = Units.degreesToRadians(15);

  public static final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
  static {
    pivotConfig.Slot0.kP = 15.0; // TODO tune
    pivotConfig.Slot0.kD = 0.001; // TODO tune

    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    pivotConfig.CurrentLimits.SupplyCurrentLimit = 30;
    pivotConfig.CurrentLimits.SupplyCurrentThreshold = 50;
    pivotConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  }

  // Tune on real robot
  // May be unnecessary depending on how OP the motor is
  public static final ArmFeedforward pivotFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);

  // Intake Constants

  public static final int intakeID = 11;

  public static final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

  static {
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Low to prevent cube popping
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 5;
    intakeConfig.CurrentLimits.SupplyCurrentThreshold = 20;
    intakeConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  }
}
