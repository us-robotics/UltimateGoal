package org.firstinspires.ftc.teamcode.TeleOps;

import org.firstinspires.ftc.teamcode.Behaviors.MecanumDrivetrain;

import java.util.List;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.TeleOp.TeleOpModeBase;

public class MecanumI3 extends TeleOpModeBase
{
	@Override
	public void addBehaviors(List<Behavior> behaviorList)
	{
		behaviorList.add(new MecanumDrivetrain(this));
	}
}
