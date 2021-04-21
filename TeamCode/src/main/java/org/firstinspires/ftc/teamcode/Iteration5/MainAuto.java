package org.firstinspires.ftc.teamcode.Iteration5;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Behaviors.DrivetrainI5;
import org.firstinspires.ftc.teamcode.Behaviors.InertialMeasurementUnit;
import org.firstinspires.ftc.teamcode.Behaviors.Intake;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.Behaviors.TeleOpSequences;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleDraggerI5;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabberI5;

import java.util.List;

import FTCEngine.Core.Auto.ConfigOption;
import FTCEngine.Core.Behavior;
import FTCEngine.Core.OpModeBase;

@Autonomous(name = "Ultimate Goal MainAuto")
public class MainAuto extends OpModeBase
{
	@Override
	protected void addBehaviors(List<Behavior> behaviorList)
	{
		behaviorList.add(new DrivetrainI5(this));
		behaviorList.add(new Launcher(this));
		behaviorList.add(new Intake(this));
		behaviorList.add(new WobbleGrabberI5(this));
		behaviorList.add(new WobbleDraggerI5(this));
		behaviorList.add(new InertialMeasurementUnit(this));
		behaviorList.add(new TeleOpSequences(this));
	}

	@Override
	protected void appendConfigOptions(List<ConfigOption> options)
	{

	}


	@Override
	public void start()
	{
		super.start();
		assignSequence(new SequenceA(this));
	}
}
