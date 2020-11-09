package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autos.TestAuto;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.Behaviors.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.WobblyGrabber;

import java.util.List;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.TeleOp.TeleOpModeBase;

@TeleOp(name = "MecanumI3 (Holden)")
public class MecanumI3 extends TeleOpModeBase
{
	@Override //add teleop behaviors in here
	public void addBehaviors(List<Behavior> behaviorList)
	{
		behaviorList.add(new MecanumDrivetrain(this));
		behaviorList.add(new Launcher(this));
		behaviorList.add(new WobblyGrabber(this));
	}
}
