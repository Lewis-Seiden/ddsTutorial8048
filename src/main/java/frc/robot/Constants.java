// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class Constants {
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

  // Pivot Constants

  public static final double pivotRatio = 1.0 / 1.0;
  public static final double pivotAllowableError = Units.degreesToRadians(15);

  // Tune on real robot
  // May be unnecessary depending on how OP the motor is
  public static final ArmFeedforward pivotFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);

  // CAN IDs

  public static final int gyroID = 0;
  public static final int pivotID = 10;
  public static final int pivotEncoderID = 0;

}
