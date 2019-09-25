
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;


@Autonomous(name = "AutoooTest")

public class AutoooTest extends LinearOpMode {

    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private org.firstinspires.ftc.teamcode.Moverobot MyRobo = null;


    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize motors
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");


        telemetry.addData("hi", "hi");
        telemetry.update();


        waitForStart();

        While(opModeIsActive()){
            MyRobo = new org.firstinspires.ftc.teamcode.Moverobot(FL, FR, BR, BL);
            MyRobo.move();





        }




    }



}
