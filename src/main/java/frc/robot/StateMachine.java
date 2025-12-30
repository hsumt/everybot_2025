package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

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
import frc.robot.subsystems.IntRollersS.rollerConstants;
import edu.wpi.first.units.measure.Distance;

import frc.robot.subsystems.ClimbS;
import frc.robot.subsystems.ClimbS.climbConstants;
import frc.robot.subsystems.IntPivotS.intakeConstants;;

public class StateMachine {
    // TODO: add logging/simulation for states

    public final ClimbS climbPivot;
    public final IntPivotS intPivot;
    public final IntRollersS intRollers;
    public final CommandSwerveDrivetrain drivetrain;
    private Autos autos;

    public enum RobotState {
        // Todo: add all states as in button mapping doc
        CORAL_PRESCORE,
        ALGAE_PRESCORE,
        DEFAULT

    }

    public RobotState currentState = RobotState.DEFAULT;
    public StateMachine(ClimbS climbPivot, IntPivotS intPivot, IntRollersS intRollers, CommandSwerveDrivetrain drivetrain) {
        this.climbPivot = climbPivot;
        this.intPivot = intPivot;
        this.intRollers = intRollers;
        this.drivetrain = drivetrain;
        this.autos = null;
    }
    public Command setState(RobotState newState) {
        return new InstantCommand(() -> currentState = newState);
    }
    public void setAutos(Autos autos) {
        this.autos = autos;
    }
    public Command AlgaeIntake() {
        return intRollers.setVoltage(rollerConstants.ALG_VOLTAGE_IN);
    }
    public Command AlgaeOuttake() {
        return intRollers.setVoltage(rollerConstants.ALG_VOLTAGE_OUT);
    }
    public Command ArmDown() {
        return intPivot.setAngle(intakeConstants.DEPLOY_LIMIT);
    }
    public Command ArmUp() {
        return intPivot.setAngle(intakeConstants.STOW_LIMIT);
    }
    public Command ClimbDeploy() {
        return climbPivot.setAngle(climbConstants.DEPLOY_LIMIT);
    }
    public Command ClimbStow() {
        return climbPivot.setAngle(climbConstants.STOW_LIMIT);
    }
    public Command CoralOuttake() {
        return intRollers.setVoltage(rollerConstants.COR_VOLTAGE_OUT);
    }
    public Command CoralStack() {
        return intRollers.setVoltage(rollerConstants.COR_VOLTAGE_OUT_STACK);
    }

    public Command AlgaeScore() {
        return Commands.race(AlgaeOuttake(), waitSeconds(0.5).andThen(ArmDown()));
    }
    public Command AlgaeObtain() {
        return Commands.parallel(ArmDown(), AlgaeIntake());
    }

}
