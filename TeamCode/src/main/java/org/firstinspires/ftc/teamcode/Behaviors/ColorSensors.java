package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Mathf;
import FTCEngine.Math.Vector2;
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

		opMode.debug.addData("Baseline", baseLine);
	}

	private ColorSensor left;
	private ColorSensor right;

	private Vector3 baseLine = Vector3.zero;
	private int baseLineCount;

	@Override
	public void update()
	{
		super.update();

		Vector3 leftDifference = getColor(left).sub(baseLine);
		Vector3 rightDifference = getColor(right).sub(baseLine);

		opMode.debug.addData("Baseline", baseLine);

		opMode.debug.addData("Left Difference", leftDifference + " " + getSaturationBrightness(leftDifference));
		opMode.debug.addData("Right Difference", rightDifference + " " + getSaturationBrightness(leftDifference));
	}

	public Line getLineUpper()
	{
		return getLine(right);
	}

	public Line getLineLower()
	{
		return getLine(left);
	}

	private Line getLine(ColorSensor sensor)
	{
		Vector3 difference = getColor(sensor).sub(baseLine);
		Vector2 sb = getSaturationBrightness(difference); //Saturation and brightness/value

		final float SaturationThreshold = 50f;
		final float BrightnessThreshold = 20f;

		if (sb.x > SaturationThreshold) return Line.COLOR;
		return sb.y > BrightnessThreshold ? Line.WHITE : Line.NONE;
	}

	private static Vector2 getSaturationBrightness(Vector3 color)
	{
		float cmax = Math.max(color.x, Math.max(color.y, color.z)) / 255f;
		float cmin = Math.min(color.x, Math.min(color.y, color.z)) / 255f;

		float saturation = Mathf.almostEquals(cmax, 0f) ? 0f : (cmax - cmin) / cmax;
		return new Vector2(saturation * 100f, cmax * 100f);
	}

	private static Vector3 getColor(ColorSensor sensor)
	{
		return new Vector3(sensor.red(), sensor.green(), sensor.blue());
	}

	public enum Line
	{
		NONE,
		COLOR,
		WHITE
	}
}
