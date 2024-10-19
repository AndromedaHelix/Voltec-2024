/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 11 02 2024
 * 
 * Manages the subsystem state machines. Acts as a robot-wide state machine that controls each mechanism's independent state machine
 */
package com.team6647.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.team6647.RobotContainer;
import com.team6647.RobotState;
import com.team6647.commands.FlywheelTarget;
import com.team6647.commands.InitIntake;
import com.team6647.commands.IntakeHome;
import com.team6647.commands.IntakeRollerTarget;
import com.team6647.commands.PrepareShooter;
import com.team6647.commands.ShooterPivotTarget;
import com.team6647.commands.ShooterRollerTarget;
import com.team6647.commands.VisionIntakeAlign;
import com.team6647.commands.VisionShuttleAlign;
import com.team6647.commands.VisionSpeakerAlign;
import com.team6647.subsystems.drive.Drive;
import com.team6647.subsystems.drive.Drive.DriveMode;
import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.flywheel.ShooterSubsystem.FlywheelState;
import com.team6647.subsystems.intake.IntakeCommands;
import com.team6647.subsystems.intake.pivot.IntakePivotSubsystem;
import com.team6647.subsystems.intake.roller.IntakeSubsystem;
import com.team6647.subsystems.intake.roller.IntakeSubsystem.IntakeRollerState;
import com.team6647.subsystems.leds.LEDSubsystem;
import com.team6647.subsystems.neural.NeuralVisionSubsystem;
import com.team6647.subsystems.shooter.ShooterCommands;
import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem;
import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem.ShooterPivotState;
import com.team6647.subsystems.shooter.roller.ShooterRollerSubsystem;
import com.team6647.subsystems.shooter.roller.ShooterRollerSubsystem.ShooterFeederState;
import com.team6647.subsystems.vision.VisionSubsystem;
import com.team6647.util.Constants.DriveConstants;
import com.team6647.util.Constants.OperatorConstants;
import com.team6647.util.Constants.FieldConstants.Speaker;
import com.team6647.util.Constants.ShooterConstants;
import com.team6647.util.LoggedTunableNumber;
import com.team6647.util.AllianceFlipUtil;
import com.team6647.util.Constants;
import com.team6647.util.ShootingCalculatorUtil;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SuperStructure {

    private static SuperStructure instance;

    private static Drive andromedaSwerve = RobotContainer.andromedaSwerve;
    private static ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
    private static ShooterRollerSubsystem rollerSubsystem = RobotContainer.shooterRollerSubsystem;
    private static ShooterPivotSubsystem shooterPivotSubsystem = RobotContainer.shooterPivotSubsystem;
    private static IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
    private static IntakePivotSubsystem intakePivotSubsystem = RobotContainer.intakePivotSubsystem;
    private static VisionSubsystem visionSubsystem = RobotContainer.visionSubsystem;
    private static NeuralVisionSubsystem neuralVisionSubsystem = RobotContainer.neuralVisionSubsystem;

    @AutoLogOutput(key = "SuperStructure/State")
    public static SuperStructureState mRobotState = SuperStructureState.IDLE;
    
    //private static LoggedTunableNumber shuttleAngle = new LoggedTunableNumber("Shooter/Pivot/ShuttleAngle", 1);
    public static int shuttleAngle = ShooterConstants.shuttleAngle1;
    //private static LoggedTunableNumber shuttleRPM = new LoggedTunableNumber("Shooter/Pivot/ShuttleAngle", 2500);
    public static int shuttleRPM = ShooterConstants.shuttleRPM1;

    public static SuperStructure getInstance() {
        if (instance == null) {
            instance = new SuperStructure();
        }
        return instance;
    }

    public enum SuperStructureState {
        IDLE,
        AUTO_IDLE,
        INTAKING_COMPLETE,
        INTAKING,
        AUTO_INTAKING,
        INDEXING,
        AUTO_INDEXING,
        SUPP_INDEXING,
        AUTO_INTAKING_COMPLETE,
        AUTO_AMP,
        SHOOTING_SPEAKER,
        AUTO_SHOOTING_SPEAKER,
        SECONDARY_AUTO_SHOOTING_SPEAKER,
        SHOOTING_SUBWOOFER,
        AUTO_SHOOTING_SUBWOOFER,
        SHUTTLE,
        SHUTTLE_ALIGN,
        SCORING_AMP,
        PREPARING_AMP,
        SHOOTING_TRAP,
        SHOOTING_MOVING,
        STOPPING_CLIMB,
        INTAKE_ALIGN,
        PREPARE_CLIMB,
        CLIMBING,
        PREPARING_SHOOTER, 
        INTAKING_FORCED,
        INTAKE_SHOOT,
        INTAKE_IDLE,
        PREPARE_AUTO_SHOOTING_SUBWOOFER,
        PREPARE_AUTO_SHOOTING,
        INTAKE_HOMED, INTAKE_DONE, ERROR, READY,
        INSTANT_SHOOT, WAITING_NOTE,
        PREPARE_AMP, AUTO_ALIGN, INSTANT_SHOOT_T
    }

    public static Command update(SuperStructureState newState) {
        switch (newState) {
            case IDLE:
                return idleCommand();
            case AUTO_IDLE:
                return autoIdleCommand();
            case INTAKING_COMPLETE:
                return fullIntakingCommand();
            case INTAKING:
                return intakingCommand();
            case INTAKING_FORCED:
                return intakingForcedCommand();
            case AUTO_INTAKING:
                return autoIntakingCommand();
            case INDEXING:
                return indexingCommand();
            case SUPP_INDEXING:
                return suppIndexCommand();
            case AUTO_INDEXING:
                return autoIndexingCommand();
            case AUTO_INTAKING_COMPLETE:
                return autoFullIntakingCommand();
            case AUTO_AMP:
                return autoScoreAmp();
            case SHOOTING_SPEAKER:
                return shootingStationary();
            case AUTO_SHOOTING_SPEAKER:
                return autoShootingStationary();
            case SECONDARY_AUTO_SHOOTING_SPEAKER:
                return secondaryAutoShootingStationary();
            case SHOOTING_SUBWOOFER:
                return shootingSubwoofer2P();
            case AUTO_SHOOTING_SUBWOOFER:
                return autoShootingSubwoofer();
            case PREPARE_AUTO_SHOOTING_SUBWOOFER:
                return prepareAutoShootingSubwoofer();
            case PREPARE_AUTO_SHOOTING:
                return prepareAutoShootingStationary();
            case SCORING_AMP:
                return scoreAmp();
            case PREPARING_AMP:
                return prepareScoreAmp();
            case SHOOTING_TRAP:
                return Commands.waitSeconds(0);
            case SHOOTING_MOVING:
                return shootingWhileMoving();
            case STOPPING_CLIMB:
                return homeElevator();
            case SHUTTLE_ALIGN:
                return new VisionShuttleAlign(andromedaSwerve, visionSubsystem);
            case INTAKE_ALIGN:
                return new VisionIntakeAlign(neuralVisionSubsystem,
                        andromedaSwerve);
            case SHUTTLE:
                return sendNotes2P();
            case PREPARE_CLIMB:
                return prepareClimb();
            case CLIMBING:
                return climb();
            case PREPARING_SHOOTER:
                return new PrepareShooter(shooterSubsystem);
            case INTAKE_SHOOT:
                return IntakeShoot();
            case INTAKE_IDLE:
                return intakeIdleCommand();
            case INTAKE_HOMED:
                return intakeHomeCommand();
            case INSTANT_SHOOT:
                return instantShootCommand();
            case INSTANT_SHOOT_T:
                return instantShootTCommand();
            case WAITING_NOTE:
                return waitingNoteCommand();
            case PREPARE_AMP:
                return prepareAutoScoreAmp();
            case AUTO_ALIGN:
                return new VisionSpeakerAlign(andromedaSwerve, visionSubsystem);
            default:
                break;
        }

        return Commands.waitSeconds(0);
    }

    private static Command IntakeShoot() {
        // insh
        return Commands.sequence(
                setGoalCommand(SuperStructureState.INDEXING),
                new InstantCommand(() -> {
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), -45, 2800);//-1
                    updateShootingParameters(ampParams);
                }),
                new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                new IntakeHome(intakePivotSubsystem),
                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.INDEXING),
                Commands.deadline(
                        ShooterCommands.getShooterIntakingCommand(),
                        new IntakeRollerTarget(
                                intakeSubsystem,
                                IntakeRollerState.INTAKING)),
                new IntakeRollerTarget(
                        intakeSubsystem,
                        IntakeRollerState.STOPPED),
                Commands.waitSeconds(0.5),

                Commands.waitUntil(() -> !Constants.OperatorConstants.INTAKE_SHUTTLE.getAsBoolean()),
                Commands.parallel(
                    new InstantCommand(()->System.out.println("b")),
                    new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
            Commands.sequence(
                    new RunCommand(() -> leds.strobePurple(0.2)).withTimeout(2),
                    new RunCommand(() -> leds.solidPurple())).repeatedly());

                
    }

    private static Command autoShootingSubwoofer() {
        return Commands.deadline(
                Commands.waitUntil(() -> !shooterSubsystem.getBeamBrake()),
                Commands.sequence(
                        setGoalCommand(SuperStructureState.SHOOTING_SUBWOOFER),
                        new InstantCommand(() -> {
                            ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), -45, 3000);

                            updateShootingParameters(ampParams);
                        }),
                        Commands.parallel(
                                new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING))
                                .withTimeout(3),
                        new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING))).andThen(new WaitCommand(1));
    }
    public static int autoShootingAngle = -45;

    public static boolean canShoot = false;
    private static Command prepareAutoShootingSubwoofer() {
        return Commands.sequence(
                //Commands.waitUntil(() -> shooterSubsystem.getBeamBrake()),
                Commands.sequence(
                        setGoalCommand(SuperStructureState.SHOOTING_SUBWOOFER),
                        new InstantCommand(() -> {
                            ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), autoShootingAngle, 3000);

                            updateShootingParameters(ampParams);
                        }),
                        Commands.parallel(
                                new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING))
                                .withTimeout(3)//huh? 3
                        ));
    }

    private static Command instantShootTCommand(){
        return Commands.sequence(
            new InstantCommand(()->{canShoot = false;}),
            new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.SHOOTING),
            new InstantCommand(()->{SuperStructure.hasNote=false;}),
            new WaitCommand(1)
            );
    }

    private static Command instantShootCommand(){
        return Commands.sequence(
            new InstantCommand(()->{canShoot = false;}),
            new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.SHOOTING),
            new InstantCommand(()->{SuperStructure.hasNote=false;}),
            new WaitCommand(1)
            );
    }

    private static Command setGoalCommand(SuperStructureState state) {
        return new InstantCommand(() -> mRobotState = state);
    }

    private static Command fullIntakingCommand() {
        return Commands.deadline(
                ShooterCommands.getShooterIntakingCommand(),
                setGoalCommand(SuperStructureState.INTAKING_COMPLETE),
                Commands.sequence(
                        IntakeCommands.getFullIntakeCommand()
                        ,Commands.waitSeconds(0.1)//0.3
                        ))

                .andThen(SuperStructure.update(SuperStructureState.IDLE).onlyIf(()->!OperatorConstants.INSTANT_SHOOTER.getAsBoolean()))
                .andThen(Commands.sequence(
                    SuperStructure.update(SuperStructureState.INTAKE_IDLE),
                    setGoalCommand(SuperStructureState.READY)
                ).onlyIf(()->OperatorConstants.INSTANT_SHOOTER.getAsBoolean())
                
                );

                //.andThen(SuperStructure.update(SuperStructureState.SHOOTING_SUBWOOFER));
                
                
                //.onlyIf(()->!OperatorConstants.SHOOT_SUBWOOFER.getAsBoolean())
                //.andThen(SuperStructure.update(SuperStructureState.INTAKE_IDLE));
                       

    }

    private static Command intakingCommand() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.INTAKING),
                IntakeCommands.getIntakeCommand(),
                Commands.waitSeconds(0.5))
                .andThen(SuperStructure.update(SuperStructureState.IDLE));
    }

    private static Command intakingForcedCommand() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.INTAKING_FORCED),
                IntakeCommands.getForcedIntakeCommand(),
                Commands.waitSeconds(0.5))
                .andThen(SuperStructure.update(SuperStructureState.IDLE));
    }

    private static Command autoIntakingCommand() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.AUTO_INTAKING),
                IntakeCommands.getIntakeCommand(),
                Commands.waitSeconds(0))//0.5
                .andThen(SuperStructure.update(SuperStructureState.INTAKE_IDLE));
    }

    private static Command indexingCommand() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.INDEXING),
                new IntakeHome(intakePivotSubsystem),
                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.INDEXING),
                Commands.deadline(
                        ShooterCommands.getShooterIntakingCommand(),
                        new IntakeRollerTarget(
                                intakeSubsystem,
                                IntakeRollerState.INTAKING)),
                new IntakeRollerTarget(
                        intakeSubsystem,
                        IntakeRollerState.STOPPED),
                Commands.waitSeconds(0.5))
                .andThen(SuperStructure.update(SuperStructureState.IDLE));
    }

    private static Command autoIndexingCommand() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.AUTO_INDEXING),
                new IntakeHome(intakePivotSubsystem),
                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.INDEXING),
                Commands.deadline(
                        ShooterCommands.getShooterIntakingCommand(),
                        new IntakeRollerTarget(
                                intakeSubsystem,
                                IntakeRollerState.INTAKING)),
                new IntakeRollerTarget(
                        intakeSubsystem,
                        IntakeRollerState.STOPPED),
                Commands.waitSeconds(0.5))
                .andThen(SuperStructure.update(SuperStructureState.AUTO_IDLE));
    }

    private static Command autoFullIntakingCommand() {
        return Commands.deadline(
                ShooterCommands.getShooterIntakingCommand(),
                setGoalCommand(SuperStructureState.AUTO_INTAKING_COMPLETE),
                Commands.sequence(
                        IntakeCommands.getFullIntakeCommand(),
                        Commands.waitSeconds(0.5)))
                .andThen(Commands.parallel(
                        new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.STOPPED),
                        new IntakeRollerTarget(intakeSubsystem, IntakeRollerState.STOPPED)));
    }

    private static Command idleCommand() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.IDLE),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.STOPPED),
                new InitIntake(intakePivotSubsystem),
                Commands.parallel(
                        Commands.waitSeconds(0.4).andThen(new IntakeHome(intakePivotSubsystem)),
                        new IntakeRollerTarget(intakeSubsystem, IntakeRollerState.STOPPED),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.HOMED),
                        new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.STOPPED),
                        new FlywheelTarget(shooterSubsystem, FlywheelState.STOPPED)));
    }

    private static Command intakeIdleCommand() {
        return Commands.sequence(
            
                setGoalCommand(SuperStructureState.INTAKE_IDLE),
                Commands.parallel(
                    new IntakeRollerTarget(intakeSubsystem, IntakeRollerState.STOPPED),
                    new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.STOPPED)                             
                ),
                new InitIntake(intakePivotSubsystem),
                Commands.parallel(
                        new IntakeHome(intakePivotSubsystem)      
                        ),
                setGoalCommand(SuperStructureState.INTAKE_DONE));
    }

    private static Command intakeHomeCommand() {
        return Commands.sequence(
            setGoalCommand(SuperStructureState.INTAKE_HOMED),
            new InitIntake(intakePivotSubsystem),
            Commands.waitSeconds(0.4).andThen(new IntakeHome(intakePivotSubsystem)),
            new InstantCommand(()->{neuralVisionSubsystem.isEnabled=false;})
        );
    }

    private static Command autoIdleCommand() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.AUTO_IDLE),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.STOPPED),
                new InitIntake(intakePivotSubsystem),
                Commands.parallel(
                        Commands.waitSeconds(0.4).andThen(new IntakeHome(intakePivotSubsystem)),
                        new IntakeRollerTarget(intakeSubsystem, IntakeRollerState.STOPPED),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.HOMED).withTimeout(0.1),
                        new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.STOPPED),
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING).withTimeout(0.5)));
    }

    private static Command homeElevator() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.STOPPING_CLIMB),
                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.HOMED));
    }

    private static Command shootingWhileMoving() {

        return Commands.waitSeconds(0);
    }

    private static Command shootingStationary() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.SHOOTING_SPEAKER),
                new InstantCommand(() -> {
                    ShootingParameters ampParams = ShootingCalculatorUtil.getShootingParameters(
                            RobotState.getPose(),
                            AllianceFlipUtil.apply(Speaker.centerSpeakerOpening.toTranslation2d()));

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new VisionSpeakerAlign(andromedaSwerve, visionSubsystem),
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    private static Command shootingStationary2P() {
        //Rewrite pending, make parallel be repetedly
        return Commands.sequence(
                setGoalCommand(SuperStructureState.SHOOTING_SPEAKER),
                new InstantCommand(() -> {
                    ShootingParameters ampParams = ShootingCalculatorUtil.getShootingParameters(
                            RobotState.getPose(),
                            AllianceFlipUtil.apply(Speaker.centerSpeakerOpening.toTranslation2d()));

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new VisionSpeakerAlign(andromedaSwerve, visionSubsystem),
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                Commands.deadline(Commands.waitUntil(()-> OperatorConstants.READY.getAsBoolean()),
                new InstantCommand(()->{

                    Commands.parallel(
                    new InstantCommand(() -> {
                            ShootingParameters ampParams = ShootingCalculatorUtil.getShootingParameters(
                            RobotState.getPose(),
                            AllianceFlipUtil.apply(Speaker.centerSpeakerOpening.toTranslation2d()));
                            updateShootingParameters(ampParams);
                    }),
                    new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)
                    ).repeatedly();
                    
                
                })
                ),
                
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING)
                );
    }

    private static Command autoShootingStationary() {
        return Commands.deadline(
                Commands.waitUntil(() -> !shooterSubsystem.getBeamBrake()),
                Commands.sequence(
                        setGoalCommand(SuperStructureState.AUTO_SHOOTING_SPEAKER),
                        new InstantCommand(() -> {
                            ShootingParameters ampParams = ShootingCalculatorUtil.getShootingParameters(
                                    RobotState.getPose(),
                                    AllianceFlipUtil.apply(Speaker.centerSpeakerOpening.toTranslation2d()));

                            updateShootingParameters(ampParams);
                        }),
                        Commands.parallel(
                                new VisionSpeakerAlign(andromedaSwerve, visionSubsystem),
                                new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                        new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING))).andThen(new WaitCommand(1));
    }

    private static Command prepareAutoShootingStationary() {
        
        return Commands.deadline(
                Commands.waitUntil(() -> !shooterSubsystem.getBeamBrake()),
                Commands.sequence(
                        setGoalCommand(SuperStructureState.AUTO_SHOOTING_SPEAKER),
                        new InstantCommand(() -> {
                            ShootingParameters ampParams = ShootingCalculatorUtil.getShootingParameters(
                                    RobotState.getPose(),
                                    AllianceFlipUtil.apply(Speaker.centerSpeakerOpening.toTranslation2d()));

                            updateShootingParameters(ampParams);
                        }),
                        Commands.parallel(
                                new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                        Commands.waitUntil(()-> canShoot),
                        new InstantCommand(()->{canShoot = false;}),
                        new VisionSpeakerAlign(andromedaSwerve, visionSubsystem),
                        new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING))).andThen(new WaitCommand(1));

                        
        }

    private static Command secondaryAutoShootingStationary() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.AUTO_SHOOTING_SPEAKER),
                new InstantCommand(() -> {
                    ShootingParameters ampParams = ShootingCalculatorUtil.getShootingParameters(
                            RobotState.getPose(),
                            AllianceFlipUtil.apply(Speaker.centerSpeakerOpening.toTranslation2d()));

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new VisionSpeakerAlign(andromedaSwerve, visionSubsystem),
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    private static Command shootingSubwoofer() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.SHOOTING_SUBWOOFER),
                new InstantCommand(() -> {
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), -45, 3000);

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    public static int subwooferRotation = ShooterConstants.angleSubwoofer;

    private static Command shootingSubwoofer2P() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.SHOOTING_SUBWOOFER),
                new InstantCommand(() -> {
                    /*
                    if(!OperatorConstants.GMODE2.getAsBoolean()){
                        subwooferRotation = -35             ;
                    }else{
                        if(OperatorConstants.SHOOT_SUBWOOFER.getAsBoolean()){
                            subwooferRotation = -40;
                        }
                    }
                     */
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), subwooferRotation, ShooterConstants.subwooferRPM);

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                setGoalCommand(SuperStructureState.READY),
                Commands.sequence(
                        new RunCommand(() -> leds.strobePurple(0.2)).withTimeout(2),
                        new RunCommand(() -> leds.solidPurple()))
                        .repeatedly()
                //Commands.waitUntil(()-> OperatorConstants.READY.getAsBoolean())
                
                );
                
                
               
    }

    private static final LEDSubsystem leds = LEDSubsystem.getInstance();

    private static Command sendNotes() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.SHUTTLE),
                new InstantCommand(() -> {
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), -45, 2800);//-1
                    //ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), shuttleAngle.getAsDouble(), shuttleRPM.getAsDouble());

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                Commands.sequence(
                        new RunCommand(() -> leds.strobePurple(0.2)).withTimeout(2),
                        new RunCommand(() -> leds.solidPurple())).repeatedly());
    }

    private static Command sendNotes2P() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.SHUTTLE),
                new InstantCommand(() -> {
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), shuttleAngle, shuttleRPM);//-1
                    //ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), shuttleAngle.getAsDouble(), shuttleRPM.getAsDouble());

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                Commands.sequence(
                        new RunCommand(() -> leds.strobePurple(0.2)).withTimeout(2),
                        new RunCommand(() -> leds.solidPurple())).repeatedly());
    }

    private static Command suppIndexCommand() {
        return Commands.deadline(
                Commands.waitUntil(() -> !shooterSubsystem.getBeamBrake()),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    /* Pathfinding */

    public static Command goToAmp() {
        return Commands.sequence(
                andromedaSwerve.getPathFindThenFollowPath("TeleopAmp", DriveConstants.pathFindingConstraints),
                prepareScoreAmp(),
                Commands.waitSeconds(1),
                setGoalCommand(SuperStructureState.SCORING_AMP),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    private static Command autoScoreAmp() {
        return Commands.deadline(
                Commands.waitUntil(() -> !shooterSubsystem.getBeamBrake()),
                Commands.sequence(
                        prepareScoreAmp(),
                        Commands.waitSeconds(1),
                        setGoalCommand(SuperStructureState.AUTO_AMP),
                        new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING))).andThen(new WaitCommand(1));
    }

    private static Command prepareAutoScoreAmp() {
        return Commands.sequence(
                Commands.sequence(
                        prepareScoreAmp(),
                        Commands.waitSeconds(1.5),
                        setGoalCommand(SuperStructureState.AUTO_AMP)
                        ));
    }

    private static Command prepareScoreAmp() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.PREPARING_AMP),
                new InstantCommand(() -> {
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(),
                            ShooterConstants.pivotAmpPosition,
                            ShooterConstants.flywheelAmpRPM);

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.AMP).withTimeout(2),
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING).withTimeout(2)));
    }

    private static Command scoreAmp() {
        return Commands.sequence(
                prepareScoreAmp(),
                setGoalCommand(SuperStructureState.SCORING_AMP),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    private static Command prepareClimb() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.PREPARE_CLIMB),
                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.CLIMBING));
    }

    private static Command climb() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.CLIMBING),
                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.HOMED));
    }

    /* Util */

    public static void updateShootingParameters(ShootingParameters newParameters) {
        ShooterSubsystem.updateShootingParameters(newParameters);
        ShooterPivotSubsystem.updateShootingParameters(newParameters);
    }

    public static Command autoBottomCommand() {
        return Commands.sequence(
                new InstantCommand(() -> {
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), -25, 5000);

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    public static Command autoMiddleCommand() {
        return Commands.sequence(
                new InstantCommand(() -> {
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), -45, 5000);

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    public static Command autoTopCommand() {
        return Commands.sequence(
                new InstantCommand(() -> {
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), -25, 5000);

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    public static boolean hasNote = false;

    public static Command waitingNoteCommand() {
        return Commands.sequence(
                //Commands.waitUntil(() -> new Debouncer(0.1).calculate(!intakeSubsystem.getBeamBrake()))
                Commands.waitUntil(() -> hasNote)
                );
    }


    
}
