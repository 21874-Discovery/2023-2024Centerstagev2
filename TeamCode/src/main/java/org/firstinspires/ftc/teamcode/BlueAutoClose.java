/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Template auto mode with no functionality
 */

@Autonomous(name="BlueAutoClose", group="Robot")
public class BlueAutoClose extends LinearOpMode {

    // setup dashboard
    // http://192.168.43.1:8080/dash
    private FtcDashboard dashboard;

    private DcMotor topLeft;
    private DcMotor topRight;
    private DcMotor bottomLeft;
    private DcMotor bottomRight;

    private DcMotor         intakeMotor        = null;   //port 2 - control hub


    private int step=1;
    private final double ppr=537.7;
    private final double     dgr    = 1.0 ;     // No External Gearing.
    private final double     wd   = 3.77953 ;     // For figuring circumference
    private final double     tpi         = (ppr * dgr)/(wd * Math.PI);

    private boolean startDelay=true;

    double starterBarter=0;
    double spitTime=0;

    //gets ticks per revolution

    @Override
    public void runOpMode() {

        // init dashboard and create a new multiple telemetry instance
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        topLeft=hardwareMap.get(DcMotor.class,"topLeft");
        topRight=hardwareMap.get(DcMotor.class,"topRight");
        bottomLeft=hardwareMap.get(DcMotor.class,"bottomLeft");
        bottomRight=hardwareMap.get(DcMotor.class,"bottomRight");


        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //reset encoder position

        //run

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //zero power behav

        //set position of wanted inches


        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //spits out by default
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        telemetry.addLine("Waiting for start...");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        starterBarter=System.nanoTime();

        while(opModeIsActive()) {
            if ((System.nanoTime()-starterBarter)/1000000000>5||!startDelay) {
                startDelay=false;
                if (step == 1) {
                    if (topLeft.getCurrentPosition() < 4 * tpi) {
                        //keep moving robot until 4 revolutions
                        topLeft.setPower(0.1);
                        topRight.setPower(0.1);
                        bottomLeft.setPower(0.1);
                        bottomRight.setPower(0.1);
                    } else {
                        step = 2;
                        topLeft.setPower(0);
                        topRight.setPower(0);
                        bottomLeft.setPower(0);
                        bottomRight.setPower(0);


                        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
                        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                        bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);


                        //init code for step 2
                    }
                }
                if (step == 2) {
                    if (topLeft.getCurrentPosition() < 30*tpi) {
                        //keep moving robot until 4 revolutions
                        topLeft.setPower(0.1);
                        topRight.setPower(0.1);
                        bottomLeft.setPower(0.1);
                        bottomRight.setPower(0.1);
                    } else {
                        step = 3;

                        topLeft.setPower(0);
                        topRight.setPower(0);
                        bottomLeft.setPower(0);
                        bottomRight.setPower(0);


                        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                        spitTime = System.nanoTime();


                    }
                }
                if (step == 3) {
                    //3 seconds
                    if ((System.nanoTime() - spitTime) / 1000000000 < 2) {
                        intakeMotor.setPower(0.4);
                    } else {
                        step = 4;

                        intakeMotor.setPower(0.0);
                    }
                }


                telemetry.addData("Average Tick data: ", (topLeft.getCurrentPosition() + topRight.getCurrentPosition() + bottomLeft.getCurrentPosition() + bottomRight.getCurrentPosition()) / 4);
                telemetry.addData("Calculated Average Inches: ", ((Math.abs(topLeft.getCurrentPosition()) + Math.abs(topRight.getCurrentPosition()) + Math.abs(bottomLeft.getCurrentPosition()) + Math.abs(bottomRight.getCurrentPosition())) / 4) / tpi);

                telemetry.update();
            }
        }
        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);
        intakeMotor.setPower(0);
    }
}
