/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 11 02 2024
 * 
 * Manages the subsystem state machines. Acts as a robot-wide state machine that controls each mechanism's independent state machine
 */
package com.team6647.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.andromedalib.util.AllianceFlipUtil;
import com.team6647.RobotContainer;
import com.team6647.commands.FlywheelTarget;
import com.team6647.commands.InitIntake;
import com.team6647.commands.IntakeHome;
import com.team6647.commands.IntakeRollerTarget;
import com.team6647.commands.ShooterPivotTarget;
import com.team6647.commands.ShooterRollerTarget;
import com.team6647.commands.ShootingStationary;
import com.team6647.commands.VisionIntakeAlign;
import com.team6647.commands.VisionSpeakerAlign;
import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.flywheel.ShooterSubsystem.FlywheelState;
import com.team6647.subsystems.intake.IntakeCommands;
import com.team6647.subsystems.intake.pivot.IntakePivotSubsystem;
import com.team6647.subsystems.intake.roller.IntakeSubsystem;
import com.team6647.subsystems.intake.roller.IntakeSubsystem.IntakeRollerState;
import com.team6647.subsystems.neural.NeuralVisionSubsystem;
import com.team6647.subsystems.shooter.ShooterCommands;
import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem;
import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem.ShooterPivotState;
import com.team6647.subsystems.shooter.roller.ShooterRollerSubsystem;
import com.team6647.subsystems.shooter.roller.ShooterRollerSubsystem.ShooterFeederState;
import com.team6647.subsystems.vision.VisionSubsystem;
import com.team6647.util.Constants.FieldConstants;
import com.team6647.util.Constants.FieldConstants.Speaker;
import com.team6647.util.Constants.ShooterConstants;
import com.team6647.util.ShootingCalculatorUtil;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SuperStructure {

    private static SuperStructure instance;

    private static AndromedaSwerve andromedaSwerve = RobotContainer.andromedaSwerve;
    private static ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
    private static ShooterRollerSubsystem rollerSubsystem = RobotContainer.shooterRollerSubsystem;
    private static ShooterPivotSubsystem shooterPivotSubsystem = RobotContainer.shooterPivotSubsystem;
    private static IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
    private static IntakePivotSubsystem intakePivotSubsystem = RobotContainer.intakePivotSubsystem;
    private static VisionSubsystem visionSubsystem = RobotContainer.visionSubsytem;
    private static NeuralVisionSubsystem neuralVisionSubsystem = RobotContainer.neuralVisionSubsystem;

    @AutoLogOutput(key = "SuperStructure/State")
    private static SuperStructureState mRobotState = SuperStructureState.IDLE;

    public static SuperStructure getInstance() {
        if (instance == null) {
            instance = new SuperStructure();
        }
        return instance;
    }

    public enum SuperStructureState {
        IDLE,
        INTAKING,
        SHOOTING_SPEAKER,
        SHOOTING_SUBWOOFER,
        INTELLIGENT_SHOOTING_SPEAKER,
        SCORING_AMP,
        SHOOTING_TRAP,
        SHOOTING_MOVING,
        STOPPING_CLIMB,
        INTAKE_ALIGN
    }

    public static Command update(SuperStructureState newState) {
        switch (newState) {
            case IDLE:
                mRobotState = SuperStructureState.IDLE;
                return idleCommand();
            case INTAKING:
                mRobotState = SuperStructureState.INTAKING;
                return intakingCommand();
            case SHOOTING_SPEAKER:
                mRobotState = SuperStructureState.SHOOTING_SPEAKER;
                return shootingStationary();
            case SHOOTING_SUBWOOFER:
                mRobotState = SuperStructureState.SHOOTING_SUBWOOFER;
                return shootingSubwoofer();
            case SCORING_AMP:
                mRobotState = SuperStructureState.SCORING_AMP;
                return scoreAmp();
            case SHOOTING_TRAP:
                return Commands.waitSeconds(0);
            case SHOOTING_MOVING:
                mRobotState = SuperStructureState.SHOOTING_MOVING;
                return shootingWhileMoving();
            case STOPPING_CLIMB:
                mRobotState = SuperStructureState.STOPPING_CLIMB;
                return homeElevator();
            case INTAKE_ALIGN:
                mRobotState = SuperStructureState.INTAKE_ALIGN;
                return new VisionIntakeAlign(neuralVisionSubsystem,
                        andromedaSwerve);
            case INTELLIGENT_SHOOTING_SPEAKER:
                mRobotState = SuperStructureState.INTELLIGENT_SHOOTING_SPEAKER;
                return intelligentShooting();
            default:
                break;
        }

        return Commands.waitSeconds(0);
    }

    private static Command intakingCommand() {

        return Commands.deadline(
                ShooterCommands.getShooterIntakingCommand(),
                Commands.sequence(
                        IntakeCommands.getIntakeCommand(),
                        Commands.waitSeconds(0.5)))
                .andThen(SuperStructure.update(SuperStructureState.IDLE));
    }

    private static Command idleCommand() {

        return Commands.sequence(
                new InitIntake(intakePivotSubsystem),
                Commands.parallel(
                        Commands.waitSeconds(0.4).andThen(new IntakeHome(intakePivotSubsystem)),
                        new IntakeRollerTarget(intakeSubsystem, IntakeRollerState.STOPPED),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.HOMED),
                        new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.STOPPED),
                        new FlywheelTarget(shooterSubsystem, FlywheelState.STOPPED)));
    }

    private static Command homeElevator() {
        return Commands.sequence(
                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.HOMED));
    }

    private static Command shootingWhileMoving() {

        return Commands.waitSeconds(0);
    }

    private static Command shootingStationary() {
        return Commands.sequence(
                new InstantCommand(() -> {
                    ShootingParameters ampParams = ShootingCalculatorUtil.getShootingParameters(
                            andromedaSwerve.getPose(),
                            AllianceFlipUtil.apply(Speaker.centerSpeakerOpening.toTranslation2d()));

                    updateShootingParameters(ampParams);
                }),
                new VisionSpeakerAlign(andromedaSwerve, visionSubsystem),
                Commands.parallel(
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    private static Command intelligentShooting() {
        return Commands.sequence(
                Commands.either(
                        Commands.sequence(
                                new ShooterRollerTarget(
                                        rollerSubsystem,
                                        ShooterFeederState.INTAKING),
                                Commands.waitUntil(() -> !shooterSubsystem.getBeamBrake())),
                        Commands.waitSeconds(0),
                        () -> shooterSubsystem.getBeamBrake()),
                new ShooterRollerTarget(rollerSubsystem,
                        ShooterFeederState.STOPPED),
                Commands.sequence(
                        new InstantCommand(() -> {
                            ShootingParameters ampParams = ShootingCalculatorUtil.getShootingParameters(
                                    andromedaSwerve.getPose(),
                                    AllianceFlipUtil.apply(Speaker.centerSpeakerOpening.toTranslation2d()));

                            updateShootingParameters(ampParams);
                        }),
                        new VisionSpeakerAlign(andromedaSwerve, visionSubsystem),
                        Commands.parallel(
                                new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                        new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING)));
    }

    private static Command shootingSubwoofer() {
        return Commands.sequence(
                new InstantCommand(() -> {
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), 110, 5000);

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    /* Pathfinding */

    public static Command goToAmp() {
        return andromedaSwerve.getPathFindPath(AllianceFlipUtil.apply(FieldConstants.amp));
    }

    private static Command scoreAmp() {
        return Commands.sequence(
                new InstantCommand(() -> {
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(),
                            ShooterConstants.pivotAmpPosition,
                            ShooterConstants.flywheelAmpRPM);

                    updateShootingParameters(ampParams);
                }),
                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.AMP).withTimeout(1),
                new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING).withTimeout(1),
                Commands.waitSeconds(1),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    /* Util */

    public static void updateShootingParameters(ShootingParameters newParameters) {
        ShooterSubsystem.updateShootingParameters(newParameters);
        ShooterPivotSubsystem.updateShootingParameters(newParameters);
    }

}
