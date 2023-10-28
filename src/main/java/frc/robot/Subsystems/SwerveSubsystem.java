// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class SwerveSubsystem extends SubsystemBase {
  SwerveModule[] modules;
  SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          new Translation2d(Constants.wheelBase / 2.0, Constants.trackWidth / 2.0),
          new Translation2d(Constants.wheelBase / 2.0, -Constants.trackWidth / 2.0),
          new Translation2d(-Constants.wheelBase / 2.0, Constants.trackWidth / 2.0),
          new Translation2d(-Constants.wheelBase / 2.0, -Constants.trackWidth / 2.0));
  SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(kinematics, new Rotation2d(), getModulePositions());

  Pigeon2 gyro = new Pigeon2(Constants.gyroID);
  StatusSignal<Double> heading = gyro.getYaw();

  public SwerveSubsystem(SwerveModule... modules) {
    this.modules = modules;
  }

  /**
   * A command to drive the robot
   *
   * @param x Forwards velocity in meters per second
   * @param y Rightwards velocity in meters per second
   * @param omega Counterclockwise rotation in radians per second
   * @param isOpenLoop Use feedback for drive motors when false
   */
  public CommandBase drive(
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier omega,
      boolean isOpenLoop,
      boolean fieldRelative) {
    return this.run(
        () -> {
          // Calculate where we want to be next loop
          // This mitigates the robot drifting while translating and rotating at the same time
          Pose2d velPose =
              new Pose2d(
                  x.getAsDouble() * Constants.loopTimeSeconds,
                  y.getAsDouble() * Constants.loopTimeSeconds,
                  new Rotation2d(omega.getAsDouble() * Constants.loopTimeSeconds));
          // Convert that to a motion over time
          Twist2d velTwist = new Pose2d().log(velPose);
          // Calculate needed module states
          SwerveModuleState[] states =
              kinematics.toSwerveModuleStates(
                  fieldRelative
                      // If field relative is set to true, use gyro to drive field relative
                      ? ChassisSpeeds.fromFieldRelativeSpeeds(
                          velTwist.dx / Constants.loopTimeSeconds,
                          velTwist.dy / Constants.loopTimeSeconds,
                          velTwist.dtheta / Constants.loopTimeSeconds,
                          Rotation2d.fromDegrees(heading.getValue()))
                      : new ChassisSpeeds(
                          velTwist.dx / Constants.loopTimeSeconds,
                          velTwist.dy / Constants.loopTimeSeconds,
                          velTwist.dtheta / Constants.loopTimeSeconds));
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
