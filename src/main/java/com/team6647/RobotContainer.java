/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 08 01 2023
 */
package com.team6647;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.BooleanSupplier;
import java.util.function.Function;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.andromedalib.andromedaSwerve.andromedaModule.AndromedaModuleIO;
import com.andromedalib.andromedaSwerve.andromedaModule.AndromedaModuleIOSim;
import com.andromedalib.andromedaSwerve.andromedaModule.AndromedaModuleIOTalonFX;
import com.andromedalib.andromedaSwerve.andromedaModule.GyroIO;
import com.andromedalib.andromedaSwerve.andromedaModule.GyroIOPigeon2;
import com.andromedalib.andromedaSwerve.utils.AndromedaMap;
import com.andromedalib.robot.SuperRobotContainer;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.team6647.commands.FlywheelTarget;
import com.team6647.commands.InitIntake;
import com.team6647.commands.IntakeHome;
import com.team6647.commands.IntakeRollerStartEnd;
import com.team6647.commands.ShooterPivotTarget;
import com.team6647.commands.ShooterRollerStartEnd;
import com.team6647.commands.ShooterRollerTarget;
import com.team6647.commands.VisionSpeakerAlign;
import com.team6647.subsystems.SuperStructure;
import com.team6647.subsystems.SuperStructure.SuperStructureState;
import com.team6647.subsystems.drive.Drive;
import com.team6647.subsystems.drive.Drive.DriveMode;
import com.team6647.subsystems.flywheel.ShooterIO;
import com.team6647.subsystems.flywheel.ShooterIOKraken;
import com.team6647.subsystems.flywheel.ShooterIOSim;
import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.flywheel.ShooterSubsystem.FlywheelState;
import com.team6647.subsystems.intake.pivot.IntakePivotIO;
import com.team6647.subsystems.intake.pivot.IntakePivotIOSim;
import com.team6647.subsystems.intake.pivot.IntakePivotIOSparkMaxKraken;
import com.team6647.subsystems.intake.pivot.IntakePivotSubsystem;
import com.team6647.subsystems.intake.roller.IntakeIO;
import com.team6647.subsystems.intake.roller.IntakeIOSim;
import com.team6647.subsystems.intake.roller.IntakeIOTalonFX;
import com.team6647.subsystems.intake.roller.IntakeSubsystem;
import com.team6647.subsystems.intake.roller.IntakeSubsystem.IntakeRollerState;
import com.team6647.subsystems.leds.LEDSubsystem;
import com.team6647.subsystems.neural.NeuralVisionIO;
import com.team6647.subsystems.neural.NeuralVisionIOLimelight;
import com.team6647.subsystems.neural.NeuralVisionSubsystem;
import com.team6647.subsystems.shooter.pivot.ShooterPivotIO;
import com.team6647.subsystems.shooter.pivot.ShooterPivotIOSim;
import com.team6647.subsystems.shooter.pivot.ShooterPivotIOTalonFX;
import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem;
import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem.ShooterPivotState;
import com.team6647.subsystems.shooter.roller.ShooterIORollerSim;
import com.team6647.subsystems.shooter.roller.ShooterIORollerSparkMax;
import com.team6647.subsystems.shooter.roller.ShooterRollerIO;
import com.team6647.subsystems.shooter.roller.ShooterRollerSubsystem;
import com.team6647.subsystems.shooter.roller.ShooterRollerSubsystem.ShooterFeederState;
import com.team6647.subsystems.vision.VisionSubsystem;
import com.team6647.subsystems.vision.VisionIO;
import com.team6647.subsystems.vision.VisionIOLimelight;
import com.team6647.subsystems.vision.VisionIOSim;
import com.team6647.util.AllianceFlipUtil;
import com.team6647.util.CameraServerThread;
import com.team6647.util.Constants.DriveConstants;
import com.team6647.util.Constants.OperatorConstants;
import com.team6647.util.Constants.RobotConstants;
import com.team6647.util.Constants.ShooterConstants;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer extends SuperRobotContainer {
        private static RobotContainer instance;

        public static Drive andromedaSwerve;
        public static IntakePivotSubsystem intakePivotSubsystem;
        public static IntakeSubsystem intakeSubsystem;
        public static ShooterPivotSubsystem shooterPivotSubsystem;
        public static ShooterSubsystem shooterSubsystem;
        public static ShooterRollerSubsystem shooterRollerSubsystem;
        public static VisionSubsystem visionSubsystem;
        public static NeuralVisionSubsystem neuralVisionSubsystem;
        public static RobotState robotState;

        private static final LEDSubsystem leds = LEDSubsystem.getInstance();

        public static SuperStructure superStructure;

        private static LoggedDashboardChooser<Command> autoDashboardChooser;

        SlewRateLimiter xLimiter, yLimiter, turningLimiter;

        private RobotContainer() {
        }

        public static RobotContainer getInstance() {
                if (instance == null) {
                        instance = new RobotContainer();
                }
                System.out.println("Mau was here");
                return instance;
        }

        @Override
        public void initSubsystems() {
                switch (RobotConstants.getMode()) {
                        case REAL:
                                andromedaSwerve = Drive.getInstance(
                                                new GyroIOPigeon2(DriveConstants.gyroID, "6647_CANivore"),
                                                new AndromedaModuleIO[] {
                                                                new AndromedaModuleIOTalonFX(0,
                                                                                DriveConstants.andromedModuleConfig(
                                                                                                AndromedaMap.mod1Const)),
                                                                new AndromedaModuleIOTalonFX(1,
                                                                                DriveConstants.andromedModuleConfig(
                                                                                                AndromedaMap.mod2Const)),
                                                                new AndromedaModuleIOTalonFX(2,
                                                                                DriveConstants.andromedModuleConfig(
                                                                                                AndromedaMap.mod3Const)),
                                                                new AndromedaModuleIOTalonFX(3,
                                                                                DriveConstants.andromedModuleConfig(
                                                                                                AndromedaMap.mod4Const)),
                                                }, DriveConstants.andromedaSwerveConfig);
                                intakeSubsystem = IntakeSubsystem.getInstance(new IntakeIOTalonFX());
                                SmartDashboard.putData(intakeSubsystem);
                                intakePivotSubsystem = IntakePivotSubsystem.getInstance(new IntakePivotIOSparkMaxKraken());
                                SmartDashboard.putData(intakePivotSubsystem);
                                shooterPivotSubsystem = ShooterPivotSubsystem.getInstance(new ShooterPivotIOTalonFX());
                                SmartDashboard.putData(shooterPivotSubsystem);
                                shooterSubsystem = ShooterSubsystem.getInstance(new ShooterIOKraken());
                                SmartDashboard.putData(shooterSubsystem);
                                shooterRollerSubsystem = ShooterRollerSubsystem.getInstance(new ShooterIORollerSparkMax());
                                SmartDashboard.putData(shooterRollerSubsystem);
                                visionSubsystem = VisionSubsystem.getInstance(new VisionIOLimelight());
                                SmartDashboard.putData(visionSubsystem);
                                neuralVisionSubsystem = NeuralVisionSubsystem.getInstance(new NeuralVisionIOLimelight());
                                SmartDashboard.putData(neuralVisionSubsystem);
                                //CameraServerThread.init();
                                break;
                        case SIM:
                                andromedaSwerve = Drive.getInstance(
                                                new GyroIO() {
                                                }, new AndromedaModuleIO[] {
                                                                new AndromedaModuleIOSim(0.1),
                                                                new AndromedaModuleIOSim(0.1),
                                                                new AndromedaModuleIOSim(0.1),
                                                                new AndromedaModuleIOSim(0.1),
                                                }, DriveConstants.andromedaSwerveConfig);
                                intakeSubsystem = IntakeSubsystem.getInstance(new IntakeIOSim());
                                intakePivotSubsystem = IntakePivotSubsystem.getInstance(new IntakePivotIOSim());
                                shooterPivotSubsystem = ShooterPivotSubsystem.getInstance(new ShooterPivotIOSim());
                                shooterSubsystem = ShooterSubsystem.getInstance(new ShooterIOSim());
                                shooterRollerSubsystem = ShooterRollerSubsystem.getInstance(new ShooterIORollerSim());
                                visionSubsystem = VisionSubsystem.getInstance(new VisionIOSim());
                                neuralVisionSubsystem = NeuralVisionSubsystem
                                                .getInstance(new NeuralVisionIOLimelight());
                                break;

                        default:
                                andromedaSwerve = Drive.getInstance(
                                                new GyroIO() {
                                                }, new AndromedaModuleIO[] {
                                                                new AndromedaModuleIO() {
                                                                },
                                                                new AndromedaModuleIO() {
                                                                },
                                                                new AndromedaModuleIO() {
                                                                },
                                                                new AndromedaModuleIO() {
                                                                },
                                                }, DriveConstants.andromedaSwerveConfig);
                                intakeSubsystem = IntakeSubsystem.getInstance(new IntakeIO() {
                                });

                                intakePivotSubsystem = IntakePivotSubsystem.getInstance(new IntakePivotIO() {
                                });
                                shooterPivotSubsystem = ShooterPivotSubsystem.getInstance(new ShooterPivotIO() {
                                });
                                shooterSubsystem = ShooterSubsystem.getInstance(new ShooterIO() {
                                });
                                shooterRollerSubsystem = ShooterRollerSubsystem.getInstance(new ShooterRollerIO() {
                                });
                                visionSubsystem = VisionSubsystem.getInstance(new VisionIO() {
                                });
                                neuralVisionSubsystem = NeuralVisionSubsystem.getInstance(new NeuralVisionIO() {
                                });
                                break;
                }
                superStructure = SuperStructure.getInstance();
                robotState = RobotState.getInstance();

                // -------- Auto Declaration --------

                if(true){
                
                NamedCommands.registerCommand("InitIntake",
                                new InitIntake(intakePivotSubsystem));
                NamedCommands.registerCommand("ShootSubwoofer",
                                SuperStructure.update(SuperStructureState.AUTO_SHOOTING_SUBWOOFER));
                NamedCommands.registerCommand("ShootMiddle",
                                SuperStructure.autoMiddleCommand().withTimeout(7));
                NamedCommands.registerCommand("ShootTop",
                                SuperStructure.autoTopCommand().withTimeout(7));
                NamedCommands.registerCommand("AmpScore",
                                SuperStructure.update(SuperStructureState.AUTO_AMP));
                NamedCommands.registerCommand("ShootStay",
                                SuperStructure.update(SuperStructureState.AUTO_SHOOTING_SPEAKER));
                NamedCommands.registerCommand("SecondaryShootStay",
                                SuperStructure.update(SuperStructureState.SECONDARY_AUTO_SHOOTING_SPEAKER)
                                                .withTimeout(6));
                NamedCommands.registerCommand("GrabPiece",
                                SuperStructure.update(SuperStructureState.AUTO_INTAKING_COMPLETE));
                NamedCommands.registerCommand("ExtendIntake",
                                SuperStructure.update(SuperStructureState.INTAKING_COMPLETE));//AUTO_INTAKING
                NamedCommands.registerCommand("IntakeHomed",
                                SuperStructure.update(SuperStructureState.INTAKE_HOMED).withTimeout(1)); 
                NamedCommands.registerCommand("IndexPiece",
                                SuperStructure.update(SuperStructureState.AUTO_INDEXING));
                
                NamedCommands.registerCommand("Idle",
                                SuperStructure.update(SuperStructureState.AUTO_IDLE).withTimeout(1));
                NamedCommands.registerCommand("VisionAlign",
                                SuperStructure.update(SuperStructureState.INTAKE_ALIGN));
                NamedCommands.registerCommand("SuppIndex",
                                SuperStructure.update(SuperStructureState.SUPP_INDEXING)); 

                NamedCommands.registerCommand("EnableNeural",
                                new InstantCommand(()->{neuralVisionSubsystem.isEnabled=true;}));
                NamedCommands.registerCommand("DisableNeural",
                                new InstantCommand(()->{neuralVisionSubsystem.isEnabled=false;}));


                NamedCommands.registerCommand("ShootMove", Commands.waitSeconds(0));
                NamedCommands.registerCommand("PrepareShoot", SuperStructure.update(SuperStructureState.PREPARE_AUTO_SHOOTING_SUBWOOFER));
                NamedCommands.registerCommand("PrepareAmp", SuperStructure.update(SuperStructureState.PREPARE_AMP));

                //NamedCommands.registerCommand("PrepareCamShoott", SuperStructure.update(SuperStructureState.PREPARE_AUTO_SHOOTING)); 
                NamedCommands.registerCommand("ReadyShoott", SuperStructure.update(SuperStructureState.INSTANT_SHOOT));


                NamedCommands.registerCommand("AngleSub", new InstantCommand(()->{SuperStructure.autoShootingAngle=-45;}));
                NamedCommands.registerCommand("AngleMid", new InstantCommand(()->{SuperStructure.autoShootingAngle=-30;}));

                NamedCommands.registerCommand("WaitForNote", SuperStructure.update(SuperStructureState.WAITING_NOTE));
                NamedCommands.registerCommand("AutoAlign", SuperStructure.update(SuperStructureState.AUTO_ALIGN));

                NamedCommands.registerCommand("EnableLL", new InstantCommand(()->{visionSubsystem.updateEnabled = true;}));
                NamedCommands.registerCommand("DisableLL", new InstantCommand(()->{visionSubsystem.updateEnabled = false;}));
                NamedCommands.registerCommand("Exit", new WaitCommand(10000));

                }

                autoDashboardChooser = new LoggedDashboardChooser<>("Auto chooser",
                                AutoBuilder.buildAutoChooser());

                // -------- Engame alers (Credits: 6328) --------
                Function<Double, Command> controllerRumbleCommandFactory = time -> Commands.sequence(
                                Commands.runOnce(
                                                () -> {
                                                        OperatorConstants.driverController1.getHID()
                                                                        .setRumble(RumbleType.kBothRumble, 1.0);
                                                        OperatorConstants.driverController2.getHID()
                                                                        .setRumble(RumbleType.kBothRumble, 1.0);
                                                        leds.endgameAlert = true;
                                                }),
                                Commands.waitSeconds(time),
                                Commands.runOnce(
                                                () -> {
                                                        OperatorConstants.driverController1.getHID()
                                                                        .setRumble(RumbleType.kBothRumble, 0.0);
                                                        OperatorConstants.driverController2.getHID()
                                                                        .setRumble(RumbleType.kBothRumble, 0.0);
                                                        leds.endgameAlert = false;
                                                }));
                new Trigger(
                                () -> DriverStation.isTeleopEnabled()
                                                && DriverStation.getMatchTime() > 0
                                                && DriverStation.getMatchTime() <= Math.round(25.0))
                                .onTrue(Commands.sequence(
                                                controllerRumbleCommandFactory.apply(1.0),
                                                Commands.waitSeconds(0.1),
                                                controllerRumbleCommandFactory.apply(1.0)));
                new Trigger(
                                () -> DriverStation.isTeleopEnabled()
                                                && DriverStation.getMatchTime() > 0
                                                && DriverStation.getMatchTime() <= Math.round(10.0))
                                .onTrue(
                                                Commands.sequence(
                                                                controllerRumbleCommandFactory.apply(0.2),
                                                                Commands.waitSeconds(0.1),
                                                                controllerRumbleCommandFactory.apply(0.2),
                                                                Commands.waitSeconds(0.1),
                                                                controllerRumbleCommandFactory.apply(0.2)));
                new Trigger(() -> DriverStation.isTeleopEnabled() && !shooterSubsystem.getBeamBrake())
                                .onTrue(Commands.sequence(
                                                controllerRumbleCommandFactory.apply(0.4),
                                                Commands.waitSeconds(0.1),
                                                controllerRumbleCommandFactory.apply(0.4)

                                ));
                new Trigger(() -> shooterPivotSubsystem.getMState() != ShooterPivotState.HOMED).whileTrue(
                                Commands.sequence(
                                                new RunCommand(() -> leds.strobRed(0.2)).withTimeout(2),
                                                new RunCommand(() -> leds.solidRed())).ignoringDisable(true)
                                                .repeatedly());

                new Trigger(() -> !intakeSubsystem.getBeamBrake())
                                .whileTrue(Commands.sequence(
                                                new RunCommand(() -> leds.strobeYellow(0.2)).withTimeout(2),
                                                new RunCommand(() -> leds.solidYellow())).ignoringDisable(true)
                                                .repeatedly());

                new Trigger(() -> !shooterSubsystem.getBeamBrake()/*&&!andromedaSwerve.headingInTolerance()*/)
                                .whileTrue(Commands.sequence(
                                                new RunCommand(() -> leds.strobeGreen(0.2)).withTimeout(2),
                                                new RunCommand(() -> leds.solidGreen())).ignoringDisable(true)
                                                .repeatedly());

                new Trigger(() -> shooterSubsystem.getBeamBrake() && intakeSubsystem.getBeamBrake()
                                && shooterPivotSubsystem.getMState() == ShooterPivotState.HOMED)
                                .whileTrue(Commands.sequence(
                                                new RunCommand(() -> leds.solidBlue())).ignoringDisable(true)
                                                .repeatedly());

                //new Trigger(() -> !shooterSubsystem.getBeamBrake()&&andromedaSwerve.headingInTolerance())
                                //.whileTrue(Commands.sequence(
                                                //new RunCommand(() -> leds.())).ignoringDisable(false)
                                                //.repeatedly());

                /*
                 * new Trigger(() -> DriverStation.isDisabled()).whileTrue(
                 * new RunCommand(() -> leds.rainbow()).ignoringDisable(true));
                 * 
                 * new Trigger(() -> DriverStation.isEnabled()).whileTrue(
                 * new RunCommand(() -> leds.solidBlue()).ignoringDisable(true));
                 */
                configSysIdBindings();
        }

        @Override
        public void configureBindings() {
                xLimiter = new SlewRateLimiter(andromedaSwerve.andromedaProfile.maxAcceleration);
                yLimiter = new SlewRateLimiter(andromedaSwerve.andromedaProfile.maxAcceleration);
                turningLimiter = new SlewRateLimiter(andromedaSwerve.andromedaProfile.maxAngularAcceleration);

                andromedaSwerve.setDefaultCommand(
                                andromedaSwerve.run(
                                                () -> {
                                                        double ySpeed = yLimiter
                                                                        .calculate(-OperatorConstants.driverController1
                                                                                        .getLeftY());
                                                        double xSpeed = xLimiter
                                                                        .calculate(-OperatorConstants.driverController1
                                                                                        .getLeftX());
                                                        double rotationSpeed = turningLimiter
                                                                        .calculate(-OperatorConstants.driverController1
                                                                                        .getRightX()*DriveConstants.rotationSensibility);



                                                        andromedaSwerve.acceptTeleopInputs(
                                                                        () -> xSpeed,
                                                                        () -> ySpeed,
                                                                        () -> rotationSpeed,
                                                                        () -> !OperatorConstants.driverController1
                                                                                        .leftStick()
                                                                                        .getAsBoolean(),
                                                                        () -> OperatorConstants.STRAIGHT.getAsBoolean());

                                                }));

                /* Driver 1 */

                // -------- Gyro Commands --------

                // configSysIdBindings();

                OperatorConstants.RESET_GYRO
                                .whileTrue(new InstantCommand(
                                                () -> andromedaSwerve.setGyroAngle(Rotations.of(0))));

                //OperatorConstants.GO_TO_AMP.whileTrue(SuperStructure.goToAmp())
                                //.onFalse(new InstantCommand(() -> Drive.setMDriveMode(DriveMode.TELEOP))
                                                //.andThen(SuperStructure.update(SuperStructureState.IDLE)));

                /*
                OperatorConstants.driverController1.y()
                                .whileTrue(SuperStructure.update(SuperStructureState.SHUTTLE_ALIGN))
                                .onFalse(new InstantCommand(() -> Drive.setMDriveMode(DriveMode.TELEOP))
                                                .andThen(SuperStructure.update(SuperStructureState.IDLE)));
                */

                /* Driver 2 */

                //OperatorConstants.FORCE_IDLE
                                //.whileTrue(SuperStructure.update(SuperStructureState.IDLE))
                                //.onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // -------- Superstructure --------

                // -------- Intake Commands --------

                // Complete intaking sequence
                OperatorConstants.TOGGLE_INTAKE
                                .whileTrue(SuperStructure.update(SuperStructureState.INTAKING_COMPLETE))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));//.onlyIf(()->!OperatorConstants.SHOOT_SUBWOOFER.getAsBoolean()));

                // Pass intake from intake to shooter
                OperatorConstants.INDEXING
                                .whileTrue(SuperStructure.update(SuperStructureState.INDEXING))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // Intake only, no shooter
                OperatorConstants.INTAKING_ONLY
                                .whileTrue(SuperStructure.update(SuperStructureState.INTAKING))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // Ignores Beam Break

                OperatorConstants.INTAKING_ONLY_FORCED
                                .whileTrue(SuperStructure.update(SuperStructureState.INTAKING_FORCED))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // -------- Shooter Commands --------

                OperatorConstants.PREPARE_SHOOTER
                                .whileTrue(new InstantCommand(()->{

                                                shooterSubsystem.setFlywheelState(FlywheelState.PREPARING);
                                        }
                                ));

                OperatorConstants.UNPREPARE_SHOOTER
                                .whileTrue(new InstantCommand(()->{

                                                shooterSubsystem.setFlywheelState(FlywheelState.STOPPED);
                                        }
                                ));

                OperatorConstants.SHOOT_SPEAKER
                                .whileTrue(SuperStructure.update(SuperStructureState.SHOOTING_SPEAKER))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // Subwoofer shootings
                
                OperatorConstants.SHOOT_SUBWOOFER
                                .whileTrue(
                                        Commands.sequence(
                                                new InstantCommand(()->{SuperStructure.mRobotState = SuperStructureState.SHUTTLE;}),
                                        new FlywheelTarget(shooterSubsystem, FlywheelState.PREPARING),
                                        new InstantCommand(()->{SuperStructure.mRobotState = SuperStructureState.PREPARING_SHOOTER;}),
                                        Commands.waitUntil(()->SuperStructure.mRobotState==SuperStructureState.READY)
                                        .onlyIf(()->OperatorConstants.TOGGLE_INTAKE.getAsBoolean()),
                                        
                                        //new ConditionalCommand(Commands.race(
                                          //      Commands.waitUntil(()->SuperStructure.mRobotState==SuperStructureState.INTAKE_DONE),
                                            //    Commands.waitUntil(()->SuperStructure.mRobotState==SuperStructureState.IDLE)
                                              //  ), new WaitCommand(0), ()->SuperStructure.mRobotState==SuperStructureState.INTAKING_COMPLETE),
                 
                                        //new InstantCommand(()->{SuperStructure.mRobotState = SuperStructureState.INDEXING;}),
                                        
                                        SuperStructure.update(SuperStructureState.SHOOTING_SUBWOOFER))//.onlyIf(()->!OperatorConstants.TOGGLE_INTAKE.getAsBoolean())
                                        )
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE).onlyIf(()->!OperatorConstants.TOGGLE_INTAKE.getAsBoolean()));
 
                // Shooting notes to wing

                OperatorConstants.SHUTTLE
                                .whileTrue(
                                new FlywheelTarget(shooterSubsystem, FlywheelState.PREPARING)
                                .andThen(Commands.waitUntil(()->SuperStructure.mRobotState==SuperStructureState.READY)).onlyIf(()->OperatorConstants.TOGGLE_INTAKE.getAsBoolean())
                                .andThen(SuperStructure.update(SuperStructureState.SHUTTLE)))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE).onlyIf(()->!OperatorConstants.TOGGLE_INTAKE.getAsBoolean()));

                // -------- Amp Commands --------

                OperatorConstants.TOGGLE_AMP
                                .whileTrue(SuperStructure.update(SuperStructureState.SCORING_AMP))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // -------- Helper Commands --------

                OperatorConstants.INTAKE_FEEDER
                                .whileTrue(Commands.parallel(
                                                new ShooterRollerStartEnd(shooterRollerSubsystem,
                                                                ShooterFeederState.INTAKING,
                                                                ShooterFeederState.STOPPED),
                                                new IntakeRollerStartEnd(intakeSubsystem, IntakeRollerState.INTAKING,
                                                                IntakeRollerState.STOPPED)));

                OperatorConstants.EXHAUST_FEEDER
                                .whileTrue(Commands.parallel(
                                                new ShooterRollerStartEnd(shooterRollerSubsystem,
                                                                ShooterFeederState.EXHAUSTING,
                                                                ShooterFeederState.STOPPED),
                                                new IntakeRollerStartEnd(intakeSubsystem, IntakeRollerState.EXHAUSTING,
                                                                IntakeRollerState.STOPPED)));

                OperatorConstants.EXHAUST_SHOOTER_FEEDER
                                .whileTrue(
                                        new InstantCommand(()->{
                                                new ShooterRollerTarget(shooterRollerSubsystem, ShooterFeederState.INTAKING);}))
                                        .onFalse(new InstantCommand(()->{
                                                new ShooterRollerTarget(shooterRollerSubsystem, ShooterFeederState.STOPPED);
                                        }));

                OperatorConstants.INTAKE_SHOOTER_FEEDER
                                .whileTrue(
                                        new InstantCommand(()->{
                                                new ShooterRollerTarget(shooterRollerSubsystem, ShooterFeederState.EXHAUSTING);}))
                                        .onFalse(new InstantCommand(()->{
                                                new ShooterRollerTarget(shooterRollerSubsystem, ShooterFeederState.STOPPED);
                                        }));

                // -------- Climbing --------

                OperatorConstants.PREPARE_CLIMB
                                .whileTrue(SuperStructure.update(SuperStructureState.PREPARE_CLIMB))
                                .and(OperatorConstants.CLIMB
                                                .whileTrue(SuperStructure.update(SuperStructureState.CLIMBING)));
                // -------- Re enabling pivot --------

                OperatorConstants.INTAKE_SHUTTLE.whileTrue(SuperStructure.update(SuperStructureState.INTAKE_SHOOT));


        

                // -------- Auto heading --------
                //55 deg
                OperatorConstants.FACE_UP.or(OperatorConstants.FACE_DOWN.or(OperatorConstants.FACE_LEFT.or(OperatorConstants.FACE_RIGHT)))
                .onTrue(
                        new InstantCommand(()->{
                        int divAmount = (OperatorConstants.FACE_DOWN.getAsBoolean()?1 : 0)+(OperatorConstants.FACE_UP.getAsBoolean()?1:0)+(OperatorConstants.FACE_LEFT.getAsBoolean()? 1 : 0)+(OperatorConstants.FACE_RIGHT.getAsBoolean()? 1 : 0);
                        Rotation2d dir = new Rotation2d((0+(OperatorConstants.FACE_DOWN.getAsBoolean()?Math.PI : 0)+(OperatorConstants.FACE_LEFT.getAsBoolean()?Math.PI/2 : 0)+(OperatorConstants.FACE_RIGHT.getAsBoolean()?-Math.PI/2 : 0))/divAmount);
                        System.out.println(dir.getDegrees());
                        System.out.println(divAmount);
                        andromedaSwerve.setTargetHeading(dir);}
                        ).andThen(
                        new InstantCommand(()->{
                        
                        Drive.setMDriveMode(DriveMode.HEADING_LOCK);
                })))
                .whileTrue(
                        new InstantCommand(()->{
                        int divAmount = (OperatorConstants.FACE_DOWN.getAsBoolean()?1 : 0)+(OperatorConstants.FACE_UP.getAsBoolean()?1:0)+(OperatorConstants.FACE_LEFT.getAsBoolean()? 1 : 0)+(OperatorConstants.FACE_RIGHT.getAsBoolean()? 1 : 0);
                        Rotation2d dir = new Rotation2d((0+(OperatorConstants.FACE_DOWN.getAsBoolean()?Math.PI : 0)+(OperatorConstants.FACE_LEFT.getAsBoolean()?Math.PI/2 : 0)+(OperatorConstants.FACE_RIGHT.getAsBoolean()?-Math.PI/2 : 0))/divAmount);
                        System.out.println(dir.getDegrees());
                        System.out.println(divAmount);
                        andromedaSwerve.setTargetHeading(dir);
                }).repeatedly()
                ).onFalse(new InstantCommand(()->{Drive.setMDriveMode(DriveMode.TELEOP);}));

                OperatorConstants.INTAKE_ALIGN.whileTrue(
                        new InstantCommand(()->{
                        andromedaSwerve.setTargetHeading(AllianceFlipUtil.apply(Rotation2d.fromDegrees(30)));
                        Drive.setMDriveMode(DriveMode.HEADING_LOCK);
                })).onFalse(
                        new InstantCommand(()->{
                        Drive.setMDriveMode(DriveMode.TELEOP);
                }));

                //public static double passAngle = 124;
                OperatorConstants.PASS_ALIGN.whileTrue(
                        new InstantCommand(()->{
                        //  Swerve/Rotation
                        andromedaSwerve.setTargetHeading(AllianceFlipUtil.apply(Rotation2d.fromDegrees(124)));
                        Drive.setMDriveMode(DriveMode.HEADING_LOCK);
                })).onFalse(
                        new InstantCommand(()->{
                        Drive.setMDriveMode(DriveMode.TELEOP);
                }));

                OperatorConstants.READY.whileTrue(
                        SuperStructure.update(SuperStructureState.INSTANT_SHOOT_T).onlyIf(()->SuperStructure.mRobotState != SuperStructureState.IDLE)
                ).onFalse(SuperStructure.update(SuperStructureState.IDLE))
                ;

                OperatorConstants.ITEST.whileTrue(
                //SuperStructure.update(SuperStructureState.IDLE),
                new FlywheelTarget(shooterSubsystem, FlywheelState.STOPPED)
                //new InstantCommand(()->{})        
                //new IntakeHome(intakePivotSubsystem)
                );

        

                

                /* 
                OperatorConstants.FACE_UP.or(OperatorConstants.FACE_DOWN.or(OperatorConstants.FACE_LEFT.or(OperatorConstants.FACE_RIGHT)))
                .onTrue(new InstantCommand(()->{Drive.setMDriveMode(DriveMode.HEADING_LOCK);}))
                .whileTrue(
                        new InstantCommand(() -> {
                                int divAmount = (OperatorConstants.FACE_DOWN.getAsBoolean()?1 : 0)+(OperatorConstants.FACE_UP.getAsBoolean()?1:0)+(OperatorConstants.FACE_LEFT.getAsBoolean()? 1 : 0)+(OperatorConstants.FACE_RIGHT.getAsBoolean()? 1 : 0);
                                Rotation2d dir = new Rotation2d(0).plus(new Rotation2d((OperatorConstants.FACE_DOWN.getAsBoolean()?Math.PI : 0))).plus(new Rotation2d((OperatorConstants.FACE_LEFT.getAsBoolean()?Math.PI/2 : 0))).plus(new Rotation2d((OperatorConstants.FACE_RIGHT.getAsBoolean()?-Math.PI/2 : 0))).times(1/divAmount);
                                andromedaSwerve.setTargetHeading(dir);
                        })
                        
                ).onFalse(new InstantCommand(()->{Drive.setMDriveMode(DriveMode.TELEOP);}));
                */

                /* 
                OperatorConstants.FACE_UP
                        .whileTrue(
                                new InstantCommand(()->{Drive.setMDriveMode(DriveMode.HEADING_LOCK);})
                                .andThen(new ConditionalCommand(
                                        new InstantCommand(() -> System.out.println("Placeholder")),
                                        new InstantCommand(() -> {andromedaSwerve.setTargetHeading(new Rotation2d(0));}),
                                        (BooleanSupplier) OperatorConstants.GMODE)))
                        .onFalse(new InstantCommand(()->{Drive.setMDriveMode(DriveMode.TELEOP);}));

                OperatorConstants.FACE_DOWN
                        .whileTrue(
                                new InstantCommand(()->{Drive.setMDriveMode(DriveMode.HEADING_LOCK);})
                                .andThen(new ConditionalCommand(
                                        new InstantCommand(() -> System.out.println("Placeholder")),
                                        new InstantCommand(() -> {andromedaSwerve.setTargetHeading(new Rotation2d(Math.PI));}),
                                        (BooleanSupplier) OperatorConstants.GMODE)))
                        .onFalse(new InstantCommand(()->{Drive.setMDriveMode(DriveMode.TELEOP);}));

                OperatorConstants.FACE_LEFT
                                .whileTrue(
                                new InstantCommand(()->{Drive.setMDriveMode(DriveMode.HEADING_LOCK);})
                                .andThen(new ConditionalCommand(
                                        new InstantCommand(() -> System.out.println("Placeholder")),
                                        new InstantCommand(() -> {andromedaSwerve.setTargetHeading(new Rotation2d(Math.PI/2));}),
                                        (BooleanSupplier) OperatorConstants.GMODE)))
                        .onFalse(new InstantCommand(()->{Drive.setMDriveMode(DriveMode.TELEOP);}));

                OperatorConstants.FACE_RIGHT
                                .whileTrue(
                                new InstantCommand(()->{Drive.setMDriveMode(DriveMode.HEADING_LOCK);})
                                .andThen(new ConditionalCommand(
                                        new InstantCommand(() -> System.out.println("Placeholder")),
                                        new InstantCommand(() -> {andromedaSwerve.setTargetHeading(new Rotation2d(-Math.PI/2));}),
                                        (BooleanSupplier) OperatorConstants.GMODE)))
                        .onFalse(new InstantCommand(()->{Drive.setMDriveMode(DriveMode.TELEOP);}));

                        */
                        
                OperatorConstants.SHOOTER_ALIGN1.or(OperatorConstants.SHOOTER_ALIGN2).whileTrue(
                        new InstantCommand(()->{visionSubsystem.updateEnabled = true;Drive.setMDriveMode(DriveMode.HEADING_LOCK);}).andThen(
                new VisionSpeakerAlign(andromedaSwerve, visionSubsystem).repeatedly()))//
                .onFalse(new InstantCommand(()->
                {Drive.setMDriveMode(DriveMode.TELEOP);}));

                OperatorConstants.DEBUG_IDLE.onTrue(
                        //SuperStructure.update(SuperStructureState.IDLE)
                        new IntakeHome(intakePivotSubsystem)
                );

                OperatorConstants.TARGET_SUBWOOFER.or(OperatorConstants.TARGET_LINE).or(OperatorConstants.TARGET_LINE).onTrue(
                        new InstantCommand(()->{
                                if(OperatorConstants.TARGET_SUBWOOFER.getAsBoolean()){
                                        //SuperStructure.subwooferRotation = ShooterConstants.angleSubwoofer;
                                        SuperStructure.shuttleAngle = ShooterConstants.shuttleAngle1;
                                        SuperStructure.shuttleRPM = ShooterConstants.shuttleRPM1;
                                }
                                if(OperatorConstants.TARGET_LINE.getAsBoolean()){
                                        //SuperStructure.subwooferRotation = ShooterConstants.angleLine;
                                        SuperStructure.shuttleAngle = ShooterConstants.shuttleAngle2;
                                        SuperStructure.shuttleRPM = ShooterConstants.shuttleRPM2;
                                }
                                if(OperatorConstants.TARGET_FAR.getAsBoolean()){
                                        //SuperStructure.subwooferRotation = ShooterConstants.angleFar;
                                        SuperStructure.shuttleAngle = ShooterConstants.shuttleAngle3;
                                        SuperStructure.shuttleRPM = ShooterConstants.shuttleRPM3;
                                }
                        })/*.andThen(
                                new InstantCommand(() -> {
                                        ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), SuperStructure.subwooferRotation, ShooterConstants.subwooferRPM);
                                        SuperStructure.updateShootingParameters(ampParams);}).andThen(
                                Commands.sequence(
                                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING)
                                ).onlyIf(()->SuperStructure.mRobotState==SuperStructureState.READY||SuperStructure.mRobotState==SuperStructureState.SHOOTING_SUBWOOFER)
                        )
                        )*/

                );
        
        }

        public void configSysIdBindings() {
                // OperatorConstants.FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER
                // .whileTrue(shooterPivotSubsystem.sysIdQuasistatic(Direction.kForward));
                // OperatorConstants.BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER
                // .whileTrue(shooterPivotSubsystem.sysIdQuasistatic(Direction.kReverse));

                // OperatorConstants.FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER
                // .whileTrue(shooterPivotSubsystem.sysIdDynamic(Direction.kForward));
                // OperatorConstants.BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER
                // .whileTrue(shooterPivotSubsystem.sysIdDynamic(Direction.kReverse));
        }

        public void configTuningBindings() {
                OperatorConstants.driverController2.povLeft()
                                .whileTrue(new InstantCommand(
                                                () -> shooterRollerSubsystem
                                                                .setMRollerState(ShooterFeederState.INTAKING)))
                                .onFalse(new InstantCommand(
                                                () -> shooterRollerSubsystem
                                                                .setMRollerState(ShooterFeederState.STOPPED)));

        }

        @Override
        public Command getAutonomousCommand() {
                return autoDashboardChooser.get();
        }
        
        
}
