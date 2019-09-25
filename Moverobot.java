
 package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;



    public class Moverobot {


        private DcMotor FR = null;
        private DcMotor FL = null;
        private DcMotor BL = null;
        private DcMotor BR = null;
        private Opmode Op = null;
        static final double TICKS_PER_REV = 1680;    // eg: TETRIX Motor Encoder
        static final double DRIVE_GEAR_REDUCTION = 1.7;     // This is < 1.0 if geared UP
        static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        static final double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        static final double DRIVE_SPEED = 0.6;
        static final double TURN_SPEED = 0.5;
        private ElapsedTime runtime = new ElapsedTime();

        private double rfLastEncoder;
        private double rbLastEncoder;
        private double lfLastEncoder;
        private double lbLastEncoder;


        public Moverobot(DcMotor rf, DcMotor rb, DcMotor lf, DcMotor lb, Telemetry telemetry) {

            this.FR = rf;
            this.BR = rb;
            this.FL = lf;
            this.BL = lb;
            this.telemetry = telemetry;
        }


        public void move ( double speed, double leftInches, double rightInches, double timeoutS){

            int newLeftFrontTarget;
            int newRightFrontTarget;
            int newLeftBackTarget;
            int newRightBackTarget;

            // Ensure that the opmode is still active


            // Determine new target position, and pass to motor controller

            newLeftFrontTarget = FL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = FR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = BL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightBackTarget = BR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);


            FR.setTargetPosition(newRightFrontTarget);
            FL.setTargetPosition(newLeftFrontTarget);
            BR.setTargetPosition(newRightBackTarget);
            BL.setTargetPosition(newLeftBackTarget);

            resetEncoders();
            SetRunToPosition();
            // reset the timeout time and start motion.
            runtime.reset();
            setPowerAll(Math.abs(speed), Math.abs(speed),Math.abs(speed),Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < 31.0) &&
                    (FR.isBusy() && FL.isBusy() && (BR.isBusy() && (BL.isBusy())))) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", newLeftBackTarget, newRightBackTarget);


                // FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                // FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                telemetry.update();
            }

        }




    public void resetEncoders(){
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void SetRunToPosition() {
        // Turn off RUN_TO_POSITION
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPowerAll(double rfSpeed, double lfSpeed, double rbSpeed, double lbSpeed) {
        FR.setPower(rfSpeed);
        FL.setPower(rbSpeed);
        BR.setPower(lfSpeed);
        BL.setPower(lbSpeed);
    }

    public void softResetEncoder(){
        rfLastEncoder = FR.getCurrentPosition();
        lfLastEncoder = FL.getCurrentPosition();
        rbLastEncoder = BR.getCurrentPosition();
        lbLastEncoder = BL.getCurrentPosition();

    }

    public double[] getEncoderPositions(){
        double[] encoders = {FR.getCurrentPosition()-rfLastEncoder, FL.getCurrentPosition()-lfLastEncoder, BL.getCurrentPosition()-lbLastEncoder, BR.getCurrentPosition()-rbLastEncoder};
        return encoders;
    }

}







