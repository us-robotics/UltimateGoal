package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Behaviors.MecanumDrivetrain;

import java.util.List;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.TeleOp.TeleOpModeBase;

@TeleOp(name = "MecanumI3")
public class MecanumI3 extends TeleOpModeBase
{
	@Override
	public void addBehaviors(List<Behavior> behaviorList)
	{
		behaviorList.add(new MecanumDrivetrain(this));
	}
}
