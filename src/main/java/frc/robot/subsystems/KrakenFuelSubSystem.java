

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.FuelConstants.INDEXER_MOTOR_ID;
import static frc.robot.Constants.KrakenFuelConstants.*;

// Subsystem stuff

public class KrakenFuelSubSystem extends SubsystemBase {
    private final TalonFX KLeftIntakeLauncher;
    private final TalonFX KRightIntakeLauncher;
    private final SparkMax Indexer;

    private final DutyCycleOut leftRequest = new DutyCycleOut(0);
    private final DutyCycleOut rightRequest = new DutyCycleOut(0);

    public KrakenFuelSubSystem() {
        KLeftIntakeLauncher  = new TalonFX(KLeftIntakeLauncherID);
        KRightIntakeLauncher  = new TalonFX(KRightIntakeLauncherID);
        Indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushed);

        // Hopefully motor doesn't tear itself apart :D

        configureKrakenMotor(KRightIntakeLauncher, InvertedValue.Clockwise_Positive);
        configureKrakenMotor(KLeftIntakeLauncher, InvertedValue.CounterClockwise_Positive);
        configureIndexer();
    }

    private void configureKrakenMotor(TalonFX motor, InvertedValue invertedValue) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode =  NeutralModeValue.Coast;
        config.MotorOutput.Inverted =  invertedValue;

        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2;

        motor.getConfigurator().apply(config);
    }

    private void configureIndexer() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);

        Indexer.configure(
            config,
            com.revrobotics.ResetMode.kNoResetSafeParameters,
            com.revrobotics.PersistMode.kNoPersistParameters
        );
    }

    // Beyond this is all running motors I think

    public void setLauncherIntakePercent(double output) {
        KLeftIntakeLauncher.setControl(leftRequest.withOutput(output));
        KRightIntakeLauncher.setControl(rightRequest.withOutput(output));
    }

    public void stopLauncherIntake() {
        setLauncherIntakePercent(0.0);
    }

    public void setIndexerPercent(double output) {
        Indexer.set(output);
    }

    public void stopIndexer() {
        Indexer.set(0.0);
    }

    public void stopAll() {
        stopLauncherIntake();
        stopIndexer();
    }
    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}