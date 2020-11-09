package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.List;
import FTCEngine.Core.Auto.AutoOpModeBase;
import FTCEngine.Core.Behavior;

@Autonomous(name = "Test Auto") //name
public class TestAuto extends AutoOpModeBase 
{

    @Override
    public void addBehaviors(List<Behavior> behaviorList) {
        behaviorList.add(new Drivetrain(this));
    }

    Drivetrain drivetrain;

    @Override
    protected void queueJobs() {
        drivetrain = getBehavior(Drivetrain.class);
    }

}
