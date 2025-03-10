package frc.robot.subsystems;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.Algae.AlgaeModule;

public class AlgaeSubsystem {
    private final AlgaeModule m_algae = new AlgaeModule(AlgaeConstants.kSparkCANid, AlgaeConstants.kFlexCANId);

    public void SetRotateState(double speed) {
        m_algae.SetRotateState(speed);
    }

    public void SetSpinState(double speed) {
        m_algae.SetSpinState(speed);
    }
}
