package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SubsystemWithMapleSimSimulation extends SubsystemBase {

    public Robot Robot;

    public SubsystemWithMapleSimSimulation(Robot robotInstance)
    {
        this.Robot = robotInstance;
    }

    @Override
    public void simulationPeriodic() {
        
        if (RobotBase.isSimulation())
        {
            periodicSimulated();
        }

    }

    @Override
    public void periodic() {

        super.periodic();

        periodicAny();
       
        periodicReal();

    }

    protected abstract void periodicReal();
    protected abstract void periodicSimulated();
    protected abstract void periodicAny();
}
