/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.util;

import com.andromedalib.andromedaSwerve.config.AndromedaModuleConfig;
import com.andromedalib.andromedaSwerve.config.AndromedaModuleConfig.ModuleMotorConfig;
import com.andromedalib.andromedaSwerve.config.AndromedaSwerveConfig;
import com.andromedalib.andromedaSwerve.config.AndromedaSwerveConfig.Mode;
import com.andromedalib.andromedaSwerve.utils.AndromedModuleIDs;
import com.andromedalib.andromedaSwerve.utils.AndromedaMap;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Constants {

        public static class OperatorConstants {

                public static final double controllerDeadband = 0.1;

                public static final int kDriverControllerPort = 0;
                public static final int kDriverControllerPort2 = 1;

                public static final CommandXboxController driverController1 = new CommandXboxController(
                                OperatorConstants.kDriverControllerPort);
                public static final CommandXboxController driverController2 = new CommandXboxController(
                                OperatorConstants.kDriverControllerPort2);
                
                private static final Trigger NONE = new Trigger(() -> false);

                /* Driver 1 */
                public static final Trigger //FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = driverController1.y(),
                                //BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = driverController1.povDownLeft(),
                                //FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = driverController1.povRight(),
                                //BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = driverController1.povLeft(),
                                RESET_GYRO = driverController1.povDown(),

                                //GMODE1 = driverController1.rightBumper(),
                                FACE_UP = driverController1.a(),
                                FACE_DOWN = driverController1.y(),
                                FACE_LEFT = driverController1.b(),
                                FACE_RIGHT = driverController1.x(),

                                SHOOTER_ALIGN1 = driverController1.rightBumper().or(driverController1.back()),
                                INTAKE_ALIGN = NONE,//driverController1.leftTrigger(),
                                PASS_ALIGN = NONE, //driverController1.rightTrigger(),
                                
                                STRAIGHT = driverController1.leftBumper(),
                                DEBUG_IDLE = driverController1.povUp();

                /* Driver 2 */

                public static final Trigger
                                GMODE2 = driverController2.back(),

                                TOGGLE_INTAKE = driverController2.povRight(),
                                INDEXING = driverController2.povUp(),
                                INTAKE_SHUTTLE = driverController2.povDown(),
                                INTAKING_ONLY = NONE,
                                INTAKING_ONLY_FORCED = driverController2.povLeft(),

                                TARGET_FAR = NONE,//driverController2.povUp(),
                                TARGET_LINE = new Trigger(() -> driverController1.getLeftY() > 0.6),
                                TARGET_SUBWOOFER = new Trigger(() -> driverController1.getLeftY() < -0.6),
                                
                                TOGGLE_AMP = driverController2.x(),
                                SHOOT_SPEAKER = NONE,
                                SHOOT_SUBWOOFER = driverController2.a(), //SHOOT_SUBWOOFER = driverController2.a(),
                                READY = driverController2.b(),
                                SHUTTLE = driverController2.y(),
                                
                                CLIMB_TOP = driverController2.povUp(),
                                INTAKE_FEEDER = driverController2.rightTrigger().and(GMODE2.negate()),
                                EXHAUST_FEEDER = driverController2.leftTrigger().and(GMODE2.negate()),

                                INTAKE_SHOOTER_FEEDER = driverController2.leftTrigger().and(GMODE2),
                                EXHAUST_SHOOTER_FEEDER = driverController2.rightTrigger().and(GMODE2),
                                //FORCE_IDLE = driverController2.povLeft(),
                                PREPARE_CLIMB = driverController2.leftBumper(),
                                CLIMB = driverController2.rightBumper(),
                                RE_ENABLE_PIVOT = driverController2.leftStick(),
                                //PREPARE_SHOOTER = driverController2.rightStick(),//driverController2.start();
                                PREPARE_SHOOTER = new Trigger(()->Math.abs(driverController2.getRightX())>0.2||Math.abs(driverController2.getRightY())>0.2),
                                UNPREPARE_SHOOTER = NONE,//
                                INSTANT_SHOOTER = SHOOT_SUBWOOFER.or(SHUTTLE),

                                SHOOTER_ALIGN2 = NONE,

                                ITEST = driverController2.start();

        }

        public static class RobotConstants {
                public static final Mode currentMode = Mode.REAL;

                public static final boolean tuningMode = true;

                public static final String mechanismsCANnivore = "rio";

                public static Mode getMode() {
                        if ((currentMode == Mode.SIM || currentMode == Mode.REPLAY) && RobotBase.isReal()) {
                                DriverStation.reportError("[Mode Error] Mode is set to " + currentMode.toString()
                                                + " , but the robot is running on a real robot. Changing mode to avoid issues",
                                                true);
                                return Mode.REAL;
                        }

                        if (currentMode == Mode.SIM)
                                return RobotBase.isReal() ? Mode.REAL : Mode.SIM;

                        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

                }
        }

        /**
         * Contains various field dimensions and useful reference points. Dimensions are
         * in meters, and sets
         * of corners start in the lower left moving clockwise. <b>All units in
         * Meters</b> <br>
         * <br>
         *
         * <p>
         * All translations and poses are stored with the origin at the rightmost point
         * on the BLUE
         * ALLIANCE wall.<br>
         * <br>
         * Length refers to the <i>x</i> direction (as described by wpilib) <br>
         * Width refers to the <i>y</i> direction (as described by wpilib)
         */
        public static class FieldConstants {
                public static double fieldLength = Units.inchesToMeters(651.223);
                public static double fieldWidth = Units.inchesToMeters(323.277);
                public static double wingX = Units.inchesToMeters(229.201);
                public static double podiumX = Units.inchesToMeters(126.75);
                public static double startingLineX = Units.inchesToMeters(74.111);

                // public static Pose2d amp = new Pose2d(new Translation2d(1.87, 7.52),
                // Rotation2d.fromDegrees(-90));

                public static Pose2d amp = new Pose2d(new Translation2d(1.826, 7), Rotation2d.fromDegrees(-90));

                public static Pose2d shuttlePose = new Pose2d(0.18, 7.71, new Rotation2d());

                /** Staging locations for each note */
                public static final class StagingLocations {
                        public static double centerlineX = fieldLength / 2.0;

                        // need to update
                        public static double centerlineFirstY = Units.inchesToMeters(29.638);
                        public static double centerlineSeparationY = Units.inchesToMeters(66);
                        public static double spikeX = Units.inchesToMeters(114);
                        // need
                        public static double spikeFirstY = Units.inchesToMeters(161.638);
                        public static double spikeSeparationY = Units.inchesToMeters(57);

                        public static Translation2d[] centerlineTranslations = new Translation2d[5];
                        public static Translation2d[] spikeTranslations = new Translation2d[3];

                        static {
                                for (int i = 0; i < centerlineTranslations.length; i++) {
                                        centerlineTranslations[i] = new Translation2d(centerlineX,
                                                        centerlineFirstY + (i * centerlineSeparationY));
                                }
                        }

                        static {
                                for (int i = 0; i < spikeTranslations.length; i++) {
                                        spikeTranslations[i] = new Translation2d(spikeX,
                                                        spikeFirstY + (i * spikeSeparationY));
                                }
                        }
                }

                /** Each corner of the speaker * */
                public static final class Speaker {

                        // corners (blue alliance origin)
                        public static Translation3d topRightSpeaker = new Translation3d(
                                        Units.inchesToMeters(18.055),
                                        Units.inchesToMeters(238.815),
                                        Units.inchesToMeters(83.091));

                        public static Translation3d topLeftSpeaker = new Translation3d(
                                        Units.inchesToMeters(18.055),
                                        Units.inchesToMeters(197.765),
                                        Units.inchesToMeters(83.091));

                        public static Translation3d bottomRightSpeaker = new Translation3d(0.0,
                                        Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
                        public static Translation3d bottomLeftSpeaker = new Translation3d(0.0,
                                        Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

                        /** Center of the speaker opening (blue alliance) */
                        public static Translation3d centerSpeakerOpening = (bottomLeftSpeaker.plus(bottomLeftSpeaker)).div(2);
                                        //.interpolate(topRightSpeaker, 0.5);

                        public static Translation2d TargetLocation = new Translation2d(0.06, 5.55);

                }

        }

        public static class ShooterConstants {
                public static final int shooterPivotCANCoderID = 20;
                public static final int shooterPivotLeftMotorID = 21;
                public static final int shooterPivotRightMotorID = 22;
                public static final int flywheelBottomMotorID = 23;
                public static final int flywheelTopMotorID = 24;
                public static final int shooterRollerMotorID = 25;

                public static final double bottomShooterKp = 0.44998;
                public static final double bottomShooterKi = 0.001;
                public static final double bottomShooterKd = 0.0;
                public static final double bottomShooterKs = 0.14004;
                public static final double bottomShooterKv = 0.11746;
                public static final double bottomShooterKa = 0.0029709;

                public static final double bottomShooterKf = 0.000195;

                public static final double topShooterKp = 0.46848;
                public static final double topShooterKi = 0.001;
                public static final double topShooterKd = 0.0;
                public static final double topShooterKs = 0.16512;
                public static final double topShooterKv = 0.11447;
                public static final double topShooterKa = 0.0035411;

                public static final double topShooterKf = 0.000198;

                public static final double shooterTolerance = 100.0;

                public static final double pivotKp = 50.0;
                public static final double pivotKi = 1.5;
                public static final double pivotKd = 0.0;
                public static final double pivotMaxVel = 0.6;
                public static final double pivotMaxAccel = 3.2;

                public static final double pidPositionTolerance = 0.05;
                public static final double positionTolerance = 1.5;

                public static final double pivotMaxVelocity = 1500.0;
                public static final double pivotMaxAcceleration = 1500.0;

                public static final int shooterBeamBrakeChannel = 5;
                public static final int shooterMotorCurrentLimit = 80;
                public static final int rollerMotorCurrentLimit = 80;

                public static final double shooterPivotEncoderOffset = -0.090847;
                public static final SensorDirectionValue shooterPivotEncoderInverted = SensorDirectionValue.CounterClockwise_Positive;

                public static final double pivotMinPosition = -95;
                public static final double pivotMaxPosition = 90;
                public static final double pivotHomedPosition = -85     ;
                public static final double pivotIndexingPosition = -43;
                public static final double pivotAmpPosition = 25;
                public static final double pivotClimbPosition = 5;

                public static final double flywheelAmpRPM = 500;

                public static final double rollerStoppedVelocity = 0.0;
                public static final double rollerIntakingVelocity = 0.3;
                public static final double rollerShootingVelocity = 0.6;
                public static final double rollerExhaustingVelocity = -0.1;
                public static final double rollerIdleVelocity = 0.1;

                public static final double shooterExhaustSpeed = 1000;
                public static final double shooterIdleSpeed = 5000;
                public static final double shooterStoppedSpeed = 0.0;

                public static final int forwardLimitSwitchID = 7;

                public static final double subwooferRPM = 2500;
                public static final double shootingRPM = 5000;

                public static final int angleSubwoofer = -45;
                public static final int angleLine = -35;
                public static final int angleFar = -30;

                public static final int shuttleAngle1 = -45;
                public static final int shuttleRPM1 = 2800;
                public static final int shuttleAngle2 = -37;
                public static final int shuttleRPM2 = 2800;
                public static final int shuttleAngle3 = -45;
                public static final int shuttleRPM3 = 2800;

                public static final InterpolatingDoubleTreeMap shooterPivotMap = new InterpolatingDoubleTreeMap();

                static {
                        shooterPivotMap.put(1.0936344809639507, -43.0);
                        shooterPivotMap.put(1.3647448719759743, -40.0);
                        shooterPivotMap.put(1.4662830296465148, -40.0);
                        shooterPivotMap.put(1.6591138264389274, -38.0);
                        shooterPivotMap.put(1.7600910482436123, -35.0);
                        shooterPivotMap.put(1.913340356564228, -33.0);
                        shooterPivotMap.put(2.1716554944487902, -30.0);
                        shooterPivotMap.put(2.3032792648108207, -27.0);
                        shooterPivotMap.put(2.481864300202367, -24.0);
                        shooterPivotMap.put(2.8409444270649162, -22.0);
                        shooterPivotMap.put(2.8409444270649162, -22.0);
                        shooterPivotMap.put(3.0616964133256297, -20.0);
                        shooterPivotMap.put(3.232448333251388, -20.0);
                        shooterPivotMap.put(3.4114312033790997, -19.0);
                        shooterPivotMap.put(3.5778698732172343, -18.0);
                        shooterPivotMap.put(3.826650816147822, -19.0);
                        shooterPivotMap.put(4.0869433490820315, -18.0);
                        shooterPivotMap.put(4.182890957872784, -17.0);
                        shooterPivotMap.put(4.360953069105574, -17.0);
                        shooterPivotMap.put(4.809843985735209, -16.0);
                        shooterPivotMap.put(5.0295966570759445, -15.0);
                }

                public static final InterpolatingDoubleTreeMap shooterTimeMap = new InterpolatingDoubleTreeMap();

                static {
                        // Distance, Seconds
                }
        }

        public static class ElevatorConstants {
                public static final int elevatorBoottomMotorID = 18;
                public static final int elevatorTopMotorID = 19;

                public static final int elevatorCANCoderID = 20;

                public static final double elevatorCANCoderOffset = -0.940000;

                public static final boolean elevatorTopMotorInverted = true;
                public static final boolean elevatorBottomMotorInverted = false;
                public static final int elevatorMotorCurrentLimit = 80;
                public static final double elevatorGearRatio = 24.6;

                public static final double elevatorKp = 1.79;
                public static final double elevatorKi = 0.015;
                public static final double elevatorKd = 0.0; // 0.00000000000000001;

                public static final double elevatorMinPosition = 0.24;
                public static final double elevatorMaxPosition = 44.5;
                public static final double elevatorTopPosition = 44;
                public static final double elevatorHomedPosition = 0.24;
                public static final double elevatorAmpPosition = 2;

                public static final double positionTolerance = 4;

                public static final SensorDirectionValue elevatorCANCoderSensorValue = SensorDirectionValue.CounterClockwise_Positive;
        }

        public static class IntakeConstants {
                public static final int intakeMotorID = 17;

                public static final double homedKp = 0.045;
                public static final double homedKi = 0.0;// 0.00004;
                public static final double homedKd = 0.0;

                public static final double extendedKp = 0.1;
                public static final double extendedKi = 0.0;
                public static final double extendedKd = 0.0;

                public static final double homedTolerance = 5;
                public static final double extendedTolerance = 2;

                public static final double extendedPIDMaxVelocity = 4500.0;
                public static final double extendedPIDMaxAcceleration = 4500.0;

                public static final double homedPIDMaxVelocity = 2500.0;
                public static final double homedPIDMaxAcceleration = 1500.0;

                public static final double minIntakePivotPosition = 100.400;
                public static final double maxIntakePivotPosition = 170.040;
                public static final double intakeHomedPosition = 109.00;
                public static final double intakeExtendedPosition = 166.000;

                public static final int intakePivotLeftMotorID = 14;
                public static final int intakePivotRightMotorID = 15;
                public static final int intakePushingMotor = 16;

                public static final double intakePivotEncoderPositionConversionFactor = 360;
                public static final double intakePivotEncoderZeroOffset = 0.0;

                public static final boolean intakePivotEncoderInverted = true;
                public static final boolean intakePushingMotorInverted = true;
                public static final boolean intakePivotLeftMotorInverted = false;
                public static final boolean intakePivotRightMotorInverted = true;

                public static final int intakeMotorsCurrentLimit = 30;

                public static final double intakeStoppedVelocity = 0.0;
                public static final double intakeIntakingVelocity = -0.7;
                public static final double intakeExhaustingVelocity = 0.5;
                public static final double intakeIdleVelocity = -0.1;

                public static final double pushingAcutatingPosition = 21;
                public static final double pushingHomedPosition = 2;

                public static final int pushingLimitSwitch = 9;
                public static final int intakeLimitSwitchLChannel = 1;
                public static final int intakeLimitSwitchRChannel = 3;
        }

        public static class VisionConstants {
                public static final String aprilLimeNTName = "limelight-backcam";
                public static final String neuralLimeNTName = "limelight-intake";

                public static final int speakerBlueCenterTagID = 7;
                public static final int speakerRedCenterTagID = 4;

                public static final int ampBlueTagID = 6;
                public static final int ampRedTagID = 5;

                public static final int odometryPipelineNumber = 0;
                public static final int speakerPipelineNumber = 1;
                public static final int ampPipelineNumber = 2;
        }

        public static class DriveConstants {

                public static final double rotationSensibility = 0.6;

                public static AndromedaModuleConfig andromedModuleConfig(AndromedModuleIDs moduleIDs) {
                        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
                        TalonFXConfiguration turningMotorConfig = new TalonFXConfiguration();
                        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

                        String swerveCANBus = "6647_CANivore";

                        double wheelDiameter = Units.inchesToMeters(4.0);

                        double steeringGearRatio = ((150.0 / 7.0) / 1.0);
                        double driveGearRatio = (6.75 / 1.0);

                        double turningKp = 38.0;
                        double turningKi = 0.0;
                        double turningKd = 0.0;

                        double driveKp = 0.1;
                        double driveKi = 0.0;
                        double driveKd = 0.0;
                        double driveKs = 0.1468;
                        double driveKv = 2.333;
                        double driveKa = 0.08205;

                        double openLoopRamp = 0.25;
                        double closedLoopRamp = 0.0;

                        InvertedValue driveMotorInvert;
                        InvertedValue angleMotorInvert;
                        
                        driveMotorInvert = InvertedValue.CounterClockwise_Positive;
                        angleMotorInvert = InvertedValue.Clockwise_Positive; // True
                        SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive; // False

                        /* Current Limiting */
                        int driveContinuousCurrentLimit = 35;
                        int drivePeakCurrentLimit = 60;
                        double drivePeakCurrentDuration = 0.1;
                        boolean driveEnableCurrentLimit = true;

                        int angleContinuousCurrentLimit = 25;
                        int anglePeakCurrentLimit = 40;
                        double anglePeakCurrentDuration = 0.1;
                        boolean angleEnableCurrentLimit = true;

                        /* Drive Motor Configuration */
                        driveMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
                        driveMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;

                        driveMotorConfig.Slot0.kP = driveKp;
                        driveMotorConfig.Slot0.kI = driveKi;
                        driveMotorConfig.Slot0.kD = driveKd;
                        driveMotorConfig.Slot0.kS = driveKs;
                        driveMotorConfig.Slot0.kV = driveKv;
                        driveMotorConfig.Slot0.kA = driveKa;

                        driveMotorConfig.MotorOutput.Inverted = driveMotorInvert;

                        driveMotorConfig.Audio.BeepOnBoot = true;
                        driveMotorConfig.Audio.BeepOnConfig = true;

                        driveMotorConfig.Feedback.SensorToMechanismRatio = driveGearRatio;

                        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = driveContinuousCurrentLimit;
                        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = driveEnableCurrentLimit;
                        driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = drivePeakCurrentLimit;
                        driveMotorConfig.CurrentLimits.SupplyTimeThreshold = drivePeakCurrentDuration;

                        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                        ModuleMotorConfig motorConfig = ModuleMotorConfig.FALCON_CONFIG;
                        /* CANCoder */

                        cancoderConfig.MagnetSensor.SensorDirection = canCoderInvert;

                        /* Turning Motor */

                        turningMotorConfig.Slot0.kP = turningKp;
                        turningMotorConfig.Slot0.kI = turningKi;
                        turningMotorConfig.Slot0.kD = turningKd;

                        turningMotorConfig.MotorOutput.Inverted = angleMotorInvert;

                        turningMotorConfig.Audio.BeepOnBoot = true;
                        turningMotorConfig.Audio.BeepOnConfig = true;

                        turningMotorConfig.CurrentLimits.SupplyCurrentLimit = angleContinuousCurrentLimit;
                        turningMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = angleEnableCurrentLimit;
                        turningMotorConfig.CurrentLimits.SupplyCurrentThreshold = anglePeakCurrentLimit;
                        turningMotorConfig.CurrentLimits.SupplyTimeThreshold = anglePeakCurrentDuration;

                        turningMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
                        turningMotorConfig.Feedback.SensorToMechanismRatio = steeringGearRatio;

                        turningMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

                        return new AndromedaModuleConfig(moduleIDs, driveMotorConfig, turningMotorConfig,
                                        cancoderConfig, wheelDiameter,
                                        swerveCANBus, motorConfig);
                }

                public static final double trackWidth = Units.inchesToMeters(18.5);
                public static final double wheelBase = Units.inchesToMeters(18.5);
                public static final double wheelDiameter = Units.inchesToMeters(4.0);
                public static final double wheelCircumference = wheelDiameter * Math.PI;

                /*
                 * This has to do with the robot-centric coordinate system in WPILib
                 * The convention is that +x is out front from the robot’s perspective and +y is
                 * out left of the robot (you can verify this with the right-hand rule, +z is up
                 * so counter-clockwise rotation is positive, which checks out).
                 */
                public static final Translation2d[] moduleTranslations = new Translation2d[] {
                                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(wheelBase / 2.0, trackWidth / 2.0)
                };

                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                moduleTranslations);

                public static final double maxSpeedMetersPerSecond = 4.7244;
                public static final double maxAccelerationMetersPerSecondSquared = maxSpeedMetersPerSecond * 0.85;
                public static final double maxAngularVelocityRadsPerSec = 11.5;
                public static final double maxAngularAccelerationRadsPerSec = 3.5;

                public static final AndromedaSwerveConfig andromedaSwerveConfig = new AndromedaSwerveConfig(0.1,
                                trackWidth,
                                wheelBase, swerveKinematics, moduleTranslations, maxSpeedMetersPerSecond,
                                maxAccelerationMetersPerSecondSquared,
                                maxAngularVelocityRadsPerSec,
                                maxAngularAccelerationRadsPerSec, wheelDiameter);

                public static final int gyroID = 13;

                /* Auto constants */
                public static final double translationP = 1.5;
                public static final double translationI = 0.00;
                public static final double translationD = 0.195;
                public static final double rotationP = 1;
                public static final double rotationI = 0.0;
                public static final double rotationD = 0.001;

                public static final PIDConstants translationConstants = new PIDConstants(translationP, translationI,
                                translationD);
                public static final PIDConstants rotationConstants = new PIDConstants(rotationP, rotationI,
                                rotationD);
                public static PathConstraints pathFindingConstraints = new PathConstraints(
                                3, 4,
                                edu.wpi.first.math.util.Units.degreesToRadians(560),
                                edu.wpi.first.math.util.Units.degreesToRadians(720));

                public static final HolonomicPathFollowerConfig holonomicPathConfig = new HolonomicPathFollowerConfig(
                                DriveConstants.translationConstants,
                                DriveConstants.rotationConstants,
                                maxSpeedMetersPerSecond,
                                Math.sqrt(Math.pow(trackWidth, 2) + Math.pow(trackWidth, 2)),
                                new ReplanningConfig());
        }
}
