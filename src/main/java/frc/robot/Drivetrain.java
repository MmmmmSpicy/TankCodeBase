// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

/** Represents a differential drive style drivetrain. */
public class Drivetrain {
  // public static final double kMaxSpeed = 6.7056; // meters per second
  // public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  // private static final double kTrackWidth = 0.381 * 2; // meters
  // private static final double kWheelRadius = 0.0508; // meters
  // private static final int kEncoderResolution = 4096;

  private final CANSparkMax leftLeader = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rightLeader = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(4, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private final SparkPIDController leftPIDController = leftLeader.getPIDController();
  private final SparkPIDController rightPIDController = rightLeader.getPIDController();

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(DriveConstants.kTrackWidth);

  private final DifferentialDriveOdometry odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  //private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
   * gyro.
   */
  public Drivetrain() {
    gyro.reset();

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightLeader.setInverted(true);

    leftEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    leftEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    rightEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    rightEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    // // Set the distance per pulse for the drive encoders. We can simply use the
    // // distance traveled for one rotation of the wheel divided by the encoder
    // // resolution.
    // leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    // rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    odometry =
        new DifferentialDriveOdometry(
            gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    // final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

    // final double leftOutput =
    //     leftPIDController.calculate(leftEncoder.getRate(), speeds.leftMetersPerSecond);
    // final double rightOutput =
    //     rightPIDController.calculate(rightEncoder.getRate(), speeds.rightMetersPerSecond);
    // leftLeader.setVoltage(leftOutput);
    // rightLeader.setVoltage(rightOutput);

    // leftLeader.setVoltage(leftOutput + leftFeedforward);
    // rightLeader.setVoltage(rightOutput + rightFeedforward);

    leftPIDController.setReference(speeds.leftMetersPerSecond, ControlType.kVelocity);
    rightPIDController.setReference(speeds.rightMetersPerSecond, ControlType.kVelocity);
    
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }
}
