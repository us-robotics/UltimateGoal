package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.OpModeBase;

public class DistanceSensors extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public DistanceSensors(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);
		distance = hardwareMap.get(DistanceSensor.class, "distance");
	}

	DistanceSensor distance;

	@Override
	public void update()
	{
		super.update();
		opMode.debug.addData("Distance", distance.getDistance(DistanceUnit.CM));
	}
}
