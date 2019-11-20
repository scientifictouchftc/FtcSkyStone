package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

@Autonomous(name = "RoverRuckusDrive")
public class RoverRuckusDrive extends LinearOpMode{

    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor PULLEY = null;
    private Servo MARKER = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double     TICKS_PER_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.9 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);



    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() throws InterruptedException


    {
        //Initialize motors
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /* ROVER RUKES
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        ROVER RUKES */


        telemetry.addData("hi", "hi");
        telemetry.update();

        telemetry.addData("Status" , "Ready to Run"); // Gold X pos.

        telemetry.update();




        waitForStart();

        encoderDrive(.2,12,12,30);
        strafeDrive(.1,40,40,30);
        strafeDrive(.1,-40,-40,30);
        encoderDrive(.2,12,12,30);

        //strafeDrive(.3,-12,-12,30);
        //encoderDrive(.2,-12,-12,30);

    }
    // (power)0.5 (time) 1000 is a almost perfect 90 degree turn.
    // (power) 1 (time) 1275 is almost a perfect 180 degree turn.



    //public void runOpModethrows InterruptedException

      /* public void encoderDrive(double speedleft,double speedright,
                                double leftInches, double rightInches,
                                double timeoutS)*/

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS)
    {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            telemetry.addData("Path1",  "CURRNET POSTION FRONT %7d :%7d", FL.getCurrentPosition(),  FR.getCurrentPosition());
            telemetry.addData("Path2",  "CURRNET POSTION BACK %7d :%7d ", BL.getCurrentPosition(), BR.getCurrentPosition());
            telemetry.update();

            newLeftFrontTarget = FL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = FR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = BL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = BR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            FR.setDirection(DcMotor.Direction.REVERSE);
            BR.setDirection(DcMotor.Direction.REVERSE);

            FR.setTargetPosition(newRightFrontTarget);
            FL.setTargetPosition(newLeftFrontTarget);
            BR.setTargetPosition(newRightBackTarget);
            BL.setTargetPosition(newLeftBackTarget);


            // Turn On RUN_TO_POSITION
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

          /* runtime.reset();
           BL.setPower(Math.abs(speedleft));
           BR.setPower(Math.abs(speedright));
           FL.setPower(Math.abs(speedleft));
           FR.setPower(Math.abs(speedright));*/

            // reset the timeout time and start motion.
            runtime.reset();
            FR.setPower(Math.abs(speed));
            FL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));


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
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d", newLeftBackTarget, newRightBackTarget);




                telemetry.update();





            }

            // Stop all motion;
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);



            // Turn off RUN_TO_POSITION
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            FR.setDirection(DcMotor.Direction.FORWARD);
            BR.setDirection(DcMotor.Direction.FORWARD);
            //  sleep(250);   // optional pause after each move

            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void strafeDrive(double speed,
                            double leftInches, double rightInches,
                            double timeoutS)
    {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            telemetry.addData("Path1",  "CURRNET POSTION FRONT %7d :%7d", FL.getCurrentPosition(),  FR.getCurrentPosition());
            telemetry.addData("Path2",  "CURRNET POSTION BACK %7d :%7d ", BL.getCurrentPosition(), BR.getCurrentPosition());

            telemetry.update();
            newLeftFrontTarget = FL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = FR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = BL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = BR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d", newLeftBackTarget, newRightBackTarget);

            telemetry.update();


            FR.setTargetPosition(newRightFrontTarget);
            FL.setTargetPosition(-newLeftFrontTarget);
            BR.setTargetPosition(-newRightBackTarget);
            BL.setTargetPosition(newLeftBackTarget);


            // Turn On RUN_TO_POSITION
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

          /* runtime.reset();
           BL.setPower(Math.abs(speedleft));
           BR.setPower(Math.abs(speedright));
           FL.setPower(Math.abs(speedleft));
           FR.setPower(Math.abs(speedright));*/

            // reset the timeout time and start motion.
            runtime.reset();
            FR.setPower(Math.abs(speed));
            FL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));


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
                //telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                //telemetry.addData("Path2",  "Running at %7d :%7d", newLeftBackTarget, newRightBackTarget);


                // FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                // FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                telemetry.update();


            }

            // Stop all motion;
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);



            // Turn off RUN_TO_POSITION
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }



}







