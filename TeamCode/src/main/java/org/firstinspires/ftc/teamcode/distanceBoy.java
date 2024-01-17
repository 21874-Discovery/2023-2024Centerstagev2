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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.stream.DoubleStream;


@TeleOp(name="distanceBoy", group="Linear Opmode")
public class distanceBoy extends LinearOpMode {

    private FtcDashboard dashboard;

    // init dashboard and create a new multiple telemetry instance

    // declare OpMode members


    // dawinching
    private DistanceSensor distanceSensor = null;
    // setup dashboard
    // http://192.168.43.1:8080/dash
    double hitList[]=new double[10];


    public boolean hitDetector(double list[]){
        double sum=0;


        for(int x=0;x<list.length;x+=1) {

            sum+=list[x];
        }

        double average=sum/list.length;
        double stdev=0;

        for(int x=0;x<list.length;x+=1) {
            stdev+=Math.pow(list[x]-average,2);
        }

        stdev=Math.sqrt(stdev/list.length);

        if(stdev<=1&&average>=315){
            return false;
        }
        else if(stdev>=20&&average>=100){
            return false;
        }
        else if(stdev>=50){
            return false;
        }
        else{
            return true;
        }


    }

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // init intake motors
        // init servo

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {
            /* READ INPUTS */
            // read current values of joystick
            for(int x=0;x<hitList.length;x+=1) {
                hitList[x]=distanceSensor.getDistance(DistanceUnit.INCH);
                telemetry.addData("Distance: ",hitList[x]);
            }


            telemetry.addData("OBJECT DETECTED: ", hitDetector(hitList));
            telemetry.update();

        }
    }
}