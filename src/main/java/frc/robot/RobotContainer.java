// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.Thread.State;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.therekrab.autopilot.APTarget;
import com.ctre.phoenix6.hardware.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import frc.robot.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntPivotS;
import frc.robot.subsystems.IntRollersS;
import frc.robot.subsystems.ClimbS;
import frc.robot.StateMachine;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // public final IntakePivotS intakePivot = new IntakePivotS();

    public final ClimbS climbPivot = new ClimbS();
    public final IntPivotS intPivot = new IntPivotS();
    public final IntRollersS intRollers = new IntRollersS();

    private final AutoFactory autoFactory;
    private Mechanism2d VISUALIZER;
    private final Autos autoRoutines;
    public final AutoChooser m_chooser = new AutoChooser();
    private final StateMachine stateMachine = new StateMachine(climbPivot, intPivot, intRollers, drivetrain);

    public RobotContainer() {

        drivetrain.resetOdometry(new Pose2d());
        VISUALIZER = logger.MECH_VISUALIZER;

        configureBindings();
        SmartDashboard.putData("Visualzer", VISUALIZER);

        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new Autos(drivetrain, intPivot, intRollers, climbPivot, autoFactory, this, stateMachine);
        SmartDashboard.putData("Auto Mode", m_chooser);

    }

    public double xButtonPressedTime = 0;

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
    drivetrain.applyRequest(() -> {
        if (!edu.wpi.first.wpilibj.DriverStation.isEnabled()) {
            return new SwerveRequest.Idle();
        }
        return drive
            .withVelocityX(-joystick.getLeftY() * MaxSpeed)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
    })
);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
       
        /*
         * joystick.a().onTrue(
         * stateMachine.intakeCoral());
         */
        drivetrain.registerTelemetry(logger::telemeterize);
        // Assigns button b on a zbox controller to the command "goToAngle".
        joystick.leftBumper().onTrue(stateMachine.AlgaeObtain());
        joystick.leftTrigger().onTrue(stateMachine.AlgaeScore());
        joystick.rightBumper().onTrue(stateMachine.CoralStack());
        joystick.rightTrigger().onTrue(stateMachine.CoralOuttake());

    }

    public Command getAutonomousCommand() {
        return m_chooser.selectedCommand();

    }
}
