/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 24 02 2024
 */

package com.team6647.subsystems.flywheel;

import com.andromedalib.motorControllers.SuperTalonFX;

import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team6647.util.Constants.RobotConstants;
import com.team6647.util.Constants.ShooterConstants;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterIOKraken implements ShooterIO {

        private static SuperTalonFX topFlywheelMotor = new SuperTalonFX(
                        ShooterConstants.flywheelTopMotorID,
                        GlobalIdleMode.Coast,
                        true, RobotConstants.mechanismsCANnivore);

        private static SuperTalonFX botttomFlywheelMotor = new SuperTalonFX(
                        ShooterConstants.flywheelBottomMotorID,
                        GlobalIdleMode.Coast,
                        true, RobotConstants.mechanismsCANnivore);

        private static DigitalInput shooterBeamBrake = new DigitalInput(ShooterConstants.shooterBeamBrakeChannel);

        private VelocityVoltage topFlywheelVelocityVoltage = new VelocityVoltage(0).withSlot(0);
        private VelocityVoltage bottomFlywheelVelocityVoltage = new VelocityVoltage(0).withSlot(0);

        Debouncer debounce = new Debouncer(0.2);

        public ShooterIOKraken() {
                topFlywheelMotor.getConfigurator().apply(new TalonFXConfiguration());
                botttomFlywheelMotor.getConfigurator().apply(new TalonFXConfiguration());

                setBottomPIDF(ShooterConstants.bottomShooterKp, ShooterConstants.bottomShooterKi,
                                ShooterConstants.bottomShooterKd,
                                ShooterConstants.bottomShooterKs, ShooterConstants.bottomShooterKv,
                                ShooterConstants.bottomShooterKa);

                setTopPIDF(ShooterConstants.topShooterKp, ShooterConstants.topShooterKi, ShooterConstants.topShooterKd,
                                ShooterConstants.topShooterKs, ShooterConstants.topShooterKv,
                                ShooterConstants.topShooterKa);
        }

        @Override
        public void updateInputs(ShooterIOInputs inputs) {
                inputs.bottomMotorVelocity = botttomFlywheelMotor.getVelocity().getValue() * 60;
                inputs.topMotorVelocity = topFlywheelMotor.getVelocity().getValue() * 60;

                inputs.topMotorPosition = topFlywheelMotor.getPosition().getValue();
                inputs.bottomMotorPosition = botttomFlywheelMotor.getPosition().getValue();

                inputs.bottomMotorCurrent = botttomFlywheelMotor.getStatorCurrent().getValue();
                inputs.topMotorCurrent = topFlywheelMotor.getStatorCurrent().getValue();
                inputs.bottomMotorVoltage = botttomFlywheelMotor.getMotorVoltage().getValue();
                inputs.topMotorVoltage = topFlywheelMotor.getMotorVoltage().getValue();
                inputs.topMotorTemperature = topFlywheelMotor.getDeviceTemp().getValue();
                inputs.bottomMotorTemperature = botttomFlywheelMotor.getDeviceTemp().getValue();

                inputs.beamBrake = shooterBeamBrake.get();
                ///                inputs.beamBrake = debounce.calculate(shooterBeamBrake.get());

        }

        @Override
        public void setShooterVelocity(double velocity) {

                if (velocity == 0) {
                        topFlywheelMotor.disable();
                        botttomFlywheelMotor.disable();
                } else {
                        velocity /= 60;

                        topFlywheelMotor.setControl(
                                        topFlywheelVelocityVoltage.withVelocity(velocity)
                                                        .withFeedForward(0));
                        botttomFlywheelMotor
                                        .setControl(bottomFlywheelVelocityVoltage.withVelocity(velocity)
                                                        .withFeedForward(0));

                }
        }

        @Override
        public void setTopPIDF(double p, double i, double d, double s, double v, double a) {
                TalonFXConfiguration topMotorConfig = new TalonFXConfiguration();

                topMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

                topMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

                topMotorConfig.Slot0.kP = p;
                topMotorConfig.Slot0.kI = i;
                topMotorConfig.Slot0.kD = d;
                topMotorConfig.Slot0.kS = s;
                topMotorConfig.Slot0.kV = v;
                topMotorConfig.Slot0.kA = a;

                topFlywheelMotor.getConfigurator().apply(topMotorConfig);
        }

        @Override
        public void setBottomPIDF(double p, double i, double d, double s, double v, double a) {
                TalonFXConfiguration bottomMotorConfig = new TalonFXConfiguration();

                bottomMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

                bottomMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

                bottomMotorConfig.Slot0.kP = p;
                bottomMotorConfig.Slot0.kI = i;
                bottomMotorConfig.Slot0.kD = d;
                bottomMotorConfig.Slot0.kS = s;
                bottomMotorConfig.Slot0.kV = v;
                bottomMotorConfig.Slot0.kA = a;

                botttomFlywheelMotor.getConfigurator().apply(bottomMotorConfig);
        }

        @Override
        public void runFlywheelCharacterization(double volts) {
                topFlywheelMotor.setVoltage(volts);
                botttomFlywheelMotor.setVoltage(volts);
        }
}
