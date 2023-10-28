// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  SwerveModule[] modules;
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(Constants.wheelBase / 2.0, Constants.trackWidth / 2.0),
      new Translation2d(Constants.wheelBase / 2.0, -Constants.trackWidth / 2.0),
      new Translation2d(-Constants.wheelBase / 2.0, Constants.trackWidth / 2.0),
      new Translation2d(-Constants.wheelBase / 2.0, -Constants.trackWidth / 2.0)
    );
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), getModulePositions());

  Pigeon2 gyro = new Pigeon2(Constants.gyroID);
  StatusSignal<Double> heading = gyro.getYaw();

  public SwerveSubsystem(SwerveModule... modules) {
    this.modules = modules;
  }

  /**
   * A command to drive the robot
   * @param x Forwards velocity in meters per second
   * @param y Rightwards velocity in meters per second
   * @param omega Counterclockwise rotation in radians per second
   * @param isOpenLoop Use feedback for drive motors when false
   */
  public CommandBase drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega, boolean isOpenLoop, boolean fieldRelative) {
    return this.run(() -> {
      ChassisSpeeds speeds = new ChassisSpeeds(x.getAsDouble(), y.getAsDouble(), omega.getAsDouble());
      // Could discretize here
      SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxModuleSpeed);
      for (int i = 0; i < states.length; i++) {
        modules[i].setTargetState(states[i], isOpenLoop);
      }
    });
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[modules.length];
    for (int i = 0; i < states.length; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(heading.getValue()), getModulePositions());
  }
}
