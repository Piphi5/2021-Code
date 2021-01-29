package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj.RobotBase;

public class Drivetrain extends SubsystemBase {

  WPI_TalonSRX mLeftLeader = new WPI_TalonSRX(DrivetrainConstants.kLeftSRXPort);
  WPI_TalonSRX mRightLeader = new WPI_TalonSRX(DrivetrainConstants.kRightSRXPort);
  WPI_VictorSPX mLeftFollower = new WPI_VictorSPX(DrivetrainConstants.kLeftSPXPort);
  WPI_VictorSPX mRightFollower = new WPI_VictorSPX(DrivetrainConstants.kRightSPXPort);

  TalonSRXSimCollection mLeftDriveSim = mLeftLeader.getSimCollection();
  TalonSRXSimCollection mRightDriveSim = mRightLeader.getSimCollection();

  // TODO Create Pigeon Wrapper Class
  AnalogGyro mGyro = new AnalogGyro(DrivetrainConstants.kPigeonPort);
  AnalogGyroSim mGyroSim = new AnalogGyroSim(mGyro);

  DifferentialDrive mDrive = new DifferentialDrive(mLeftLeader, mRightLeader);

  DifferentialDrivetrainSim mDriveSim = new DifferentialDrivetrainSim(
    DCMotor.getCIM(2),        //2 CIMS on each side of the drivetrain.
    DrivetrainConstants.kGearRatio,               //Standard AndyMark Gearing reduction.
    2.1,                      //MOI of 2.1 kg m^2 (from CAD model).
    26.5,                     //Mass of the robot is 26.5 kg.
    Units.inchesToMeters(DrivetrainConstants.kWheelRadiusInches),  //Robot uses 3" radius (6" diameter) wheels.
    0.546,                    //Distance between wheels is _ meters.
    
    // The standard deviations for measurement noise:
    // x and y:          0.001 m
    // heading:          0.001 rad
    // l and r velocity: 0.1   m/s
    // l and r position: 0.005 m
    null //VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this line to add measurement noise.
  );

  Field2d mField = new Field2d();
  // Creating my odometry object. Here,
  // our starting pose is 5 meters along the long end of the field and in the
  // center of the field along the short end, facing forward.
  DifferentialDriveOdometry mOdometry = new DifferentialDriveOdometry(mGyro.getRotation2d());
  


  public Drivetrain() {
    mRightLeader.configFactoryDefault();
    mRightFollower.configFactoryDefault();
    mLeftLeader.configFactoryDefault();
    mLeftFollower.configFactoryDefault();

    mRightFollower.follow(mRightLeader);
    mRightFollower.setInverted(InvertType.FollowMaster);

    mLeftFollower.follow(mLeftLeader);
    mLeftFollower.setInverted(InvertType.FollowMaster);

    if(RobotBase.isReal()){
      // On our real robot, the left side is positive forward and
      // sensor is in phase by default, so don't change it
      mLeftLeader.setInverted(InvertType.None);
      mLeftLeader.setSensorPhase(false);

      // On the real robot, the right side sensor is already in phase but
      // the right side output needs to be inverted so that positive is forward.
      mRightLeader.setInverted(InvertType.InvertMotorOutput);
      mRightLeader.setSensorPhase(false);
    } else {
      // Drive simulator expects positive motor outputs for forward
      // and returns positive encoder values, so we don't need to
      // invert or set sensor phase.
      mLeftLeader.setInverted(InvertType.None);
      mLeftLeader.setSensorPhase(false);
      mRightLeader.setInverted(InvertType.None);
      mRightLeader.setSensorPhase(false);
    }


    SmartDashboard.putData("Field", mField);
  }

  @Override
  public void periodic() {
    mOdometry.update(mGyro.getRotation2d(),
                    nativeUnitsToDistanceMeters(mLeftLeader.getSelectedSensorPosition()),
                    nativeUnitsToDistanceMeters(mRightLeader.getSelectedSensorPosition()));
    mField.setRobotPose(mOdometry.getPoseMeters());

  }

  @Override
  public void simulationPeriodic() {
    mDriveSim.setInputs(mLeftLeader.getMotorOutputVoltage(), -mRightLeader.getMotorOutputVoltage());

    mDriveSim.update(0.02);

    mLeftDriveSim.setQuadratureRawPosition(
      distanceToNativeUnits(mDriveSim.getLeftPositionMeters()));
    mRightDriveSim.setQuadratureRawPosition(
      distanceToNativeUnits(mDriveSim.getRightPositionMeters()));

    mLeftDriveSim.setQuadratureVelocity(
      velocityToNativeUnits(mDriveSim.getLeftVelocityMetersPerSecond()));
    mRightDriveSim.setQuadratureVelocity(
      velocityToNativeUnits(mDriveSim.getRightVelocityMetersPerSecond()));
    
    mGyroSim.setAngle(-mDriveSim.getHeading().getDegrees());

    mLeftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
    mRightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / DrivetrainConstants.kCountsPerRev;
    double wheelRotations = motorRotations / DrivetrainConstants.kSensorGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(DrivetrainConstants.kWheelRadiusInches));
    return positionMeters;
  }

  private int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(DrivetrainConstants.kWheelRadiusInches));
    double motorRotations = wheelRotations * DrivetrainConstants.kSensorGearRatio;
    int sensorCounts = (int)(motorRotations * DrivetrainConstants.kCountsPerRev);
    return sensorCounts;
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(DrivetrainConstants.kWheelRadiusInches));
    double motorRotationsPerSecond = wheelRotationsPerSecond * DrivetrainConstants.kSensorGearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / DrivetrainConstants.k100msPerSecond;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * DrivetrainConstants.kCountsPerRev);
    return sensorCountsPer100ms;
  }

  private double nativeUnitsToVelocity(double sensorVelocity) {
    double motorRate = (double) sensorVelocity / DrivetrainConstants.kCountsPerRev;
    double wheelRate = motorRate / DrivetrainConstants.kSensorGearRatio;
    double velocityMeters = wheelRate * (2 * Math.PI * Units.inchesToMeters(DrivetrainConstants.kWheelRadiusInches));
    return velocityMeters;
  } 

  public void arcadeDrive(double forward, double turn) {
    if (RobotBase.isReal()) {
      mDrive.arcadeDrive(-forward, turn);
    } else {
      mDrive.arcadeDrive(-forward, -turn);
    }
    
    
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    if (!RobotBase.isReal()) {
      rightVolts = -rightVolts;
    }
    mLeftLeader.setVoltage(leftVolts);
    mRightLeader.setVoltage(rightVolts);
    mDrive.feed();
  }

  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(nativeUnitsToVelocity(mLeftLeader.getSelectedSensorVelocity()), nativeUnitsToVelocity(mRightLeader.getSelectedSensorVelocity()));
  }

  public void resetOdometry(Pose2d pose) {
    mLeftLeader.setSelectedSensorPosition(0);
    mRightLeader.setSelectedSensorPosition(0);

    mOdometry.resetPosition(pose, mGyro.getRotation2d());
  }

  
}
