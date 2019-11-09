package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Encoder1Rotation")

public class Encoder1Rotation extends LinearOpMode {

    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor BR = null;


    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize motors
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");

        telemetry.addData("hi", "Encoder Test Nov");
        telemetry.update();

       

        int newLeftFrontTarget =0;
        int newRightFrontTarget=0;
        int newLeftBackTarget=0;
        int newRightBackTarget=0;

        // Determine new target position, and pass to motor controllerj6dmju6
       // newLeftFrontTarget = FL.getCurrentPosition();
       // newRightFrontTarget = FR.getCurrentPosition();
       // newLeftBackTarget = BL.getCurrentPosition();
       // newRightBackTarget = BR.getCurrentPosition();

        //int ticks = (int)(distCm / Math.cos(Math.PI/4) * COUNTS_PER_INCH);
        int ticks = 1*538;

        newLeftFrontTarget += ticks;
        newRightFrontTarget += ticks;
        newLeftBackTarget += ticks;
        newRightBackTarget += ticks;

        
        

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        resetEncoder();
        runusingEncoder();
        waitForStart();


        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setTargetPosition(newLeftFrontTarget);
        FR.setTargetPosition(newRightFrontTarget);
        BL.setTargetPosition(newLeftBackTarget);
        BR.setTargetPosition(newRightBackTarget);
        

        // reset the timeout time and start motion.
        setRunToPosition();
        //SET POWER ALWAYS SHOULD COMER AFTER RUN TO POSITION
        setPowerAll(Math.abs(0.1), Math.abs(0.1), Math.abs(0.1), Math.abs(0.1));

       


        while (opModeIsActive() && (FR.isBusy() && FL.isBusy() && BR.isBusy() && BL.isBusy()))
        {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
            telemetry.addData("Path2", "Running at %7d :%7d", newLeftBackTarget, newRightBackTarget);

            telemetry.update();
        }

        stoprobot();
        // Turn off RUN_TO_POSITION
        runusingEncoder();
        
    }

    public void runusingEncoder () {
        // Turn OFF RUN_TO_POSITION
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void resetEncoder () {
        // Turn OFF RUN_TO_POSITION
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    
    
    public void setRunToPosition () {
        // Turn ON RUN_TO_POSITION

        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void setPowerAll ( double rfSpeed, double lfSpeed, double rbSpeed, double lbSpeed){
        FR.setPower(rfSpeed);
        FL.setPower(lfSpeed);
        BR.setPower(rbSpeed);
        BL.setPower(lbSpeed);
    }

    public void stoprobot () {
        FR.setPower(0.0);
        FL.setPower(0.0);
        BR.setPower(0.0);
        BL.setPower(0.0);
    }



}
