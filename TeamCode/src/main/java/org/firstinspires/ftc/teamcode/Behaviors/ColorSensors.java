package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Vector3;

public class ColorSensors extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public ColorSensors(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		left = hardwareMap.colorSensor.get("leftColor");
		right = hardwareMap.colorSensor.get("rightColor");

		left.enableLed(true);
		right.enableLed(true);
	}

	@Override
	public void awakeUpdate()
	{
		super.awakeUpdate();

		Vector3 average = getColor(left).add(getColor(right)).div(2f);
		float count = ++baseLineCount;

		//Base line should converge towards the average
		baseLine = baseLine.mul((count - 1f) / count).add(average.div(count));
	}

	ColorSensor left;
	ColorSensor right;

	Vector3 baseLine = Vector3.zero;
	int baseLineCount;

	@Override
	public void update()
	{
		super.update();

		Vector3 leftDifference = getColor(left).sub(baseLine);
		Vector3 rightDifference = getColor(right).sub(baseLine);

		opMode.debug.addData("Left Difference", leftDifference);
		opMode.debug.addData("Right Difference", rightDifference);
	}

	private static Vector3 getColor(ColorSensor sensor)
	{
		return new Vector3(sensor.red(), sensor.green(), sensor.blue());
	}
}
