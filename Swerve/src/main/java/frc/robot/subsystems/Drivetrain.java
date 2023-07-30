// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.lib.Mk4iSwerveModuleHelper;
import frc.robot.lib.SdsModuleConfigurations;
import frc.robot.lib.SwerveModule;
import frc.robot.lib.Mk4iSwerveModuleHelper.GearRatio;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private static final class Config{
    public static final double kTrackwidth = 2;
    public static final double kWheelbase = 2;

    public static final double kMaxVoltage = 12.0;
    public static final double kMaxVelocity = 6380.0/60.0 * SdsModuleConfigurations.MK4_L2.getDriveReduction();
    public static final double kMaxAngularVelocity = kMaxVelocity / Math.hypot(kTrackwidth/2.0, kWheelbase/2.0) * 0.5;

    public static final GearRatio kGearRatio = null;
    public static final double kOffset = 1.0;

    //For module front left
    public static final int kFLDriveMotorPort = 1;
    public static final int kFLSteerMotorPort = 2;
    public static final int kFLSteerEncoderPort = 3;

    //For module front right
    public static final int kFRDriveMotorPort = 4;
    public static final int kFRSteerMotorPort = 5;
    public static final int kFRSteerEncoderPort = 6;

    //For module back left 
    public static final int kBLDriveMotorPort = 7;
    public static final int kBLSteerMotorPort = 8;
    public static final int kBLSteerEncoderPort = 9;

    //For module back right
    public static final int kBRDriveMotorPort = 10;
    public static final int kBRSteerMotorPort = 11;
    public static final int kBRSteerEncoderPort = 12;

    public static final double kAngularError = 0;

    public static final double kRotP = 0.03;
    public static final double kRotI = 0.001;
    public static final double kRotD = 0.003;
  }

  private SwerveModule[] m_swerveModules;
  private SwerveDrivePoseEstimator m_estimator;

  private final SwerveDriveKinematics m_kinematics = 
    new SwerveDriveKinematics(
      //FR
      new Translation2d(Config.kTrackwidth/2.0, -Config.kWheelbase/2.0),
      //FL
      new Translation2d(Config.kTrackwidth/2.0, -Config.kWheelbase/2.0),
      //BL
      new Translation2d(Config.kTrackwidth/2.0, -Config.kWheelbase/2.0),
      //BR
      new Translation2d(Config.kTrackwidth/2.0, -Config.kWheelbase/2.0)
      );

  private SwerveModule createModule(
    String title, GearRatio gearRatio, int driveMotorPort, int steerMotorPort, int steerEncoderPort, double steerOffset){
    return Mk4iSwerveModuleHelper.createFalcon500(
      gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort, steerOffset);
  }

  private Pose2d m_robotPose = new Pose2d();

  private Field2d m_field = new Field2d();

  private final PIDController m_rotController;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();

  private Pair<Double, Double> m_xyInput = new Pair<>(0d, 0d);
  
  private double m_targetAngle = 0;

  private ADXRS450_Gyro m_gyro  = new ADXRS450_Gyro();

  private boolean m_showShuffleboardDebugData = false;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    final SwerveModule m_frontLeft = 
    createModule("Front left", Config.kGearRatio, Config.kFLDriveMotorPort, Config.kFLSteerMotorPort, Config.kFLSteerEncoderPort, Config.kOffset);
  
    final SwerveModule m_frontRight = 
    createModule("Front right", Config.kGearRatio, Config.kFRDriveMotorPort, Config.kFRSteerMotorPort, Config.kFRSteerEncoderPort, Config.kOffset);
  
    final SwerveModule m_backLeft = 
    createModule("Back left", Config.kGearRatio, Config.kBLDriveMotorPort, Config.kBLSteerMotorPort, Config.kBLSteerEncoderPort, Config.kOffset);
 
    final SwerveModule m_backRight = 
    createModule("Front left", Config.kGearRatio, Config.kBRDriveMotorPort, Config.kBRSteerMotorPort, Config.kBRSteerEncoderPort, Config.kOffset);
  
    m_swerveModules = new SwerveModule[]{m_frontLeft, m_frontRight, m_backLeft, m_backRight};
  
    m_rotController = new PIDController(m_targetAngle, m_targetAngle, m_targetAngle);
    m_rotController.setSetpoint(0);
    m_rotController.setTolerance(Config.kAngularError);

    // m_estimator =
    //     new SwerveDrivePoseEstimator(
    //         m_kinematics,
    //         getConsistentGyroscopeRotation(),
    //         getSwerveModulePositions(),
    //         new Pose2d(3.5, 2.2, Rotation2d.fromDegrees(0)),
    //         PoseEstimator.STATE_STANDARD_DEVIATIONS,
    //         PoseEstimator.VISION_MEASUREMENT_STANDARD_DEVIATIONS);
  }

  public static double normalizeDegrees(double degrees) {
    return (degrees % 360 + 360) % 360;
  }

  public static double relativeAngularDifference(double currentAngle, double newAngle) {
    double a = normalizeDegrees(currentAngle - newAngle);
    double b = normalizeDegrees(newAngle - currentAngle);
    return a < b ? a : -b;
  }

  public static double relativeAngularDifference(Rotation2d currentAngle, double newAngle) {
    return relativeAngularDifference(currentAngle.getDegrees(), newAngle);
  }

  private SwerveModulePosition[] getSwerveModulePositions(){
    return new SwerveModulePosition[] {
      m_swerveModules[0].getPosition(),
      m_swerveModules[1].getPosition(),
      m_swerveModules[2].getPosition(),
      m_swerveModules[3].getPosition()  
    };
  }

  public Pose2d getPose() {
    return m_robotPose;
  }

  private Rotation2d m_driverGyroOffset = Rotation2d.fromDegrees(0);

  public void zeroGyroscope() {
    m_driverGyroOffset = getConsistentGyroscopeRotation();
  }

  public void smartZeroGyroscope() {
    m_driverGyroOffset =
        getConsistentGyroscopeRotation()
            .minus(
                m_estimator
                    .getEstimatedPosition()
                    .getRotation()
                    .plus(
                        DriverStation.getAlliance() == Alliance.Blue
                            ? new Rotation2d()
                            : Rotation2d.fromDegrees(180)));
  }

  public void resetOdometryToPose(Pose2d pose) {

    m_driverGyroOffset =
        getConsistentGyroscopeRotation()
            .minus(pose.getRotation())
            .plus(
                DriverStation.getAlliance() == Alliance.Blue
                    ? new Rotation2d()
                    : Rotation2d.fromDegrees(180));

    m_estimator.resetPosition(
        getConsistentGyroscopeRotation(), getSwerveModulePositions(), pose);
  }

  public Rotation2d getConsistentGyroscopeRotation() {
    return Rotation2d.fromDegrees(normalizeDegrees(-m_gyro.getAngle()));
  }

  public Rotation2d getDriverGyroscopeRotation(){
    double angle = normalizeDegrees(-m_gyro.getAngle());
    
    return Rotation2d.fromDegrees(angle).minus(m_driverGyroOffset);
  }

  public void drive(ChassisSpeeds chassisSpeeds){
    m_chassisSpeeds = chassisSpeeds;
  }
  
  public void driveAngle(Pair<Double, Double> xyInput, double targetAngle){
    m_xyInput = xyInput;
    m_targetAngle = targetAngle;
  }

  private void drivePeriodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Config.kMaxVelocity);

    // sets swerve module speeds and angles, for each swerve module, using kinematics
    for (int i = 0; i < m_swerveModules.length; i++) {
      m_swerveModules[i].set(
          states[i].speedMetersPerSecond / Config.kMaxVelocity * Config.kMaxVoltage,
          states[i].angle.getRadians());
    }
  }

  private void driveAnglePeriodic() {
    double angularDifference =
        -relativeAngularDifference(getDriverGyroscopeRotation(), m_targetAngle);

    double rotationValue = m_rotController.calculate(angularDifference);

    // we are treating this like a joystick, so -1 and 1 are its lower and upper bound
    rotationValue = MathUtil.clamp(rotationValue, -1, 1);

    // this value makes our unit-less [-1, 1] into [-max angular, max angular]
    double omegaRadiansPerSecond = rotationValue * Config.kMaxAngularVelocity;

    // initialize chassis speeds but add our desired angle
    m_chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            m_xyInput.getFirst(),
            m_xyInput.getSecond(),
            omegaRadiansPerSecond,
            getDriverGyroscopeRotation());

    // use the existing drive periodic logic to assign to motors ect
    drivePeriodic();
  }

  private double m_lastTimestamp = 0.0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final double m_timestamp = Timer.getFPGATimestamp();
    final double m_dt = m_timestamp - m_lastTimestamp;
    m_lastTimestamp = m_timestamp;

    /* get the current set-points for the drivetrain */
    Pose2d m_pose = getPose();

    m_field.setRobotPose(m_estimator.getEstimatedPosition());
    if (m_showShuffleboardDebugData) {
      SmartDashboard.putString(
          "pose",
          String.format(
              "(%2f %2f %2f)",
              m_estimator.getEstimatedPosition().getX(),
              m_estimator.getEstimatedPosition().getY(),
              m_estimator.getEstimatedPosition().getRotation().getDegrees()));
    }
  }

  public static ChassisSpeeds produceChassisSpeeds(
      boolean isRobotRelativeForward,
      boolean isRobotRelativeBackward,
      double x,
      double y,
      double rotationVelocity,
      Rotation2d currentGyroAngle) {
    if (isRobotRelativeForward) return new ChassisSpeeds(x, y, rotationVelocity);
    if (isRobotRelativeBackward) return new ChassisSpeeds(-x, -y, rotationVelocity);
    return ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotationVelocity, currentGyroAngle);
  }
}
