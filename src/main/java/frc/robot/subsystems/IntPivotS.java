
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.StateMachine;
import frc.robot.generated.TunerConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntPivotS extends SubsystemBase {
  public class intakeConstants {
    public static final Angle STOW_LIMIT = Degrees.of(95);
    public static final Angle DEPLOY_LIMIT = Degrees.of(30);
    public static final double PIV_MOTOR_CURRENT_LIMIT = 60;
    public static final int MOTOR_ID = 9;


    public static final double KP = 18;
    public static final double KI = 0;
    public static final double KD = 0.2;
    public static final double KS = -0.1;
    public static final double KG = 1.2;
    public static final double KV = 0;
    public static final double KA = 0;
    public static final double VELOCITY = 458;
    public static final double ACCELERATION = 688;
    public static final double MOI = 0.1055457256;
  }

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(intakeConstants.KP, intakeConstants.KI, intakeConstants.KD,
          DegreesPerSecond.of(intakeConstants.VELOCITY), DegreesPerSecondPerSecond.of(intakeConstants.ACCELERATION))
      .withSimClosedLoopController(intakeConstants.KP, intakeConstants.KI, intakeConstants.KD,
          DegreesPerSecond.of(intakeConstants.VELOCITY),
          DegreesPerSecondPerSecond.of(intakeConstants.ACCELERATION))
      // Feedforward Constants
      .withFeedforward(
          new ArmFeedforward(intakeConstants.KS, intakeConstants.KG, intakeConstants.KV, intakeConstants.KA))
      .withSimFeedforward(
          new ArmFeedforward(intakeConstants.KS, intakeConstants.KG, intakeConstants.KV, intakeConstants.KA))
      // Telemetry name and verbosity level
      .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which
      // corresponds to the gearbox attached to your motor.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(24,1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(intakeConstants.PIV_MOTOR_CURRENT_LIMIT));

  // Vendor motor controller object
  private TalonFX pivotMotor = new TalonFX(intakeConstants.MOTOR_ID, TunerConstants.kCANBus2);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController IntakeSMC = new TalonFXWrapper(pivotMotor, DCMotor.getKrakenX60(1), smcConfig);

  private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
      .withRelativePosition(new Translation3d(Meters.of(0.3175), Meters.of(0.2032), Meters.of(0.358775)));

  private ArmConfig armCfg = new ArmConfig(IntakeSMC)
      // Soft limit is applied to the SmartMotorControllers PID

      .withHardLimit(intakeConstants.DEPLOY_LIMIT, intakeConstants.STOW_LIMIT)
      // Starting position is where your arm starts
      .withStartingPosition(intakeConstants.STOW_LIMIT)

      // Length and mass of your arm for sim.
      .withLength(Feet.of((17.625 / 12)))

      .withMOI(intakeConstants.MOI)

      // Telemetry name and verbosity for the arm.
      .withTelemetry("Intake", TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(robotToMechanism);

  // Arm Mechanism
  private Arm arm = new Arm(armCfg);

  /**
   * Set the angle of the arm.
   * 
   * @param angle Angle to go to.
   */
  public Command setAngle(Angle angle) {

    return arm.setAngle(angle);
  }

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() {
    return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arm.updateTelemetry();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    arm.simIterate();
  }

  public Angle getAngle() {
    return arm.getAngle();
  }

  public Command pivotAngle(Angle angle) {
    return run(() -> arm.setAngle(angle));
  }
  
}