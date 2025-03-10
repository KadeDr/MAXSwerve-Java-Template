package frc.robot.subsystems.Algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;

public class AlgaeModule {
    SparkMax m_sparkMax;
    SparkFlex m_sparkFlex;

    public AlgaeModule(int sparkCANId, int flexCANId) {
        m_sparkMax = new SparkMax(sparkCANId, MotorType.kBrushless);
        m_sparkFlex = new SparkFlex(flexCANId, MotorType.kBrushless);

        m_sparkMax.configure(Configs.AlgaeModule.mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void SetRotateState(double speed) {
        m_sparkMax.set(speed);
    }

    public void SetSpinState(double speed) {
        m_sparkFlex.set(speed);
    }
}
