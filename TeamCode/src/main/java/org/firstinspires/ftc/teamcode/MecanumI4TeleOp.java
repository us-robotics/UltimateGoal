package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autos.DrivetrainAuto;
import org.firstinspires.ftc.teamcode.Autos.IntakeAuto;
import org.firstinspires.ftc.teamcode.Autos.LauncherAuto;
import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.InertialMeasurementUnit;
import org.firstinspires.ftc.teamcode.Behaviors.Intake;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.Behaviors.TeleOpSequences;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabber;

import java.util.List;

import FTCEngine.Core.Auto.ConfigOption;
import FTCEngine.Core.Behavior;
import FTCEngine.Core.OpModeBase;

@TeleOp(name = "MecanumI4 TeleOp")
public class MecanumI4TeleOp extends OpModeBase
{
	@Override //add teleop behaviors in here
	public void addBehaviors(List<Behavior> behaviorList)
	{
		behaviorList.add(new Drivetrain(this));
		behaviorList.add(new DrivetrainAuto(this));
		behaviorList.add(new Launcher(this));
		behaviorList.add(new LauncherAuto(this));
		behaviorList.add(new Intake(this));
		behaviorList.add(new IntakeAuto(this));
		behaviorList.add(new InertialMeasurementUnit(this));
		behaviorList.add(new WobbleGrabber(this));
		behaviorList.add(new TeleOpSequences(this));
//		behaviorList.add(new CameraVision(this));
	}

	@Override
	protected void appendConfigOptions(List<ConfigOption> options)
	{

	}
}
