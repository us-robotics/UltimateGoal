package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.OpModeBase;

public class Drivetrain extends AutoBehavior<Drivetrain.Job> {

    public Drivetrain(OpModeBase opMode) {
        super(opMode);
    }

    @Override
    public void awake(HardwareMap hardwareMap) { //initialize all the crap
        super.awake(hardwareMap);

        frontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        frontRight = hardwareMap.dcMotor.get("motorFrontRight");
        backLeft = hardwareMap.dcMotor.get("motorBackLeft");
        backRight = hardwareMap.dcMotor.get("motorBackRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher = hardwareMap.dcMotor.get("launcher");
    }
    //following Gary's syntax and declaring down here to separate methods
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    private DcMotor launcher;

    @Override
    protected void updateJob() {

        Job job = getCurrentJob();


    }

    static class Job extends FTCEngine.Core.Auto.Job{



    }

}
