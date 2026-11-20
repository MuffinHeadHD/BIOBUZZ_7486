package org.firstinspires.ftc.teamcode.parts

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.limelightvision.LLResultTypes
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.clamp
import org.firstinspires.ftc.teamcode.interp1D
import org.firstinspires.ftc.teamcode.rampTowards
import org.firstinspires.ftc.teamcode.slew
import kotlin.math.abs
import kotlin.math.tan

@Config
class turretPID {
    companion object {
        @JvmField
        var kD_tt = 0.0010 // 0.0005
        @JvmField
        var kP_tt = 0.014 // 0.009
        @JvmField
        var kI_tt = 0.000001 // 0.000001
        @JvmField
        var kS_tt = 0.035
        @JvmField
        var kV_tt = 0.0

    }
}

class Flywheel(val limelight: Limelight3A, val turretTop: AxonServo, val flywheelMotor: DcMotorEx) : Updatable {


    private var homePosition: Double = 0.7554545454545455

    override fun update() {
        turretTop.pid.kp = turretPID.kP_tt
        turretTop.pid.ki = turretPID.kI_tt
        turretTop.pid.kd = turretPID.kD_tt
        turretTop.pid.ks = turretPID.kS_tt
        turretTop.pid.kv = turretPID.kV_tt

        turretTop.update()
    }

    var aimTagId: Int? = null

    private var lastTxDeg = 0.0
    private var lastTyDeg = 0.0
    private var lastSeenTimeMs = System.currentTimeMillis()
    private val holdLastSeenMs = 250L

    private val deadbandDeg = 0.01
    private val minPower = 0.040
    private val maxPower_tt = 1.0
    private var lastTx = 0.0
    private var txVel = 0.0

    private val camMountDeg = 19.97
    val tagHeightCm = 74.93
    val camHeightCm = 36.02493
    private var distFiltCm = 0.0

    private val MaxVoltage = 3.3
    private val MaxDistance_mm = 1000.0

    private val distAlpha = 0.25
    private var lastDistSeenMs = System.currentTimeMillis()
    private val holdDistMs = 250L

    private val minVel = 0.0
    private val maxVel = 4500.0
    private var targetVelCmd = 2200.0
    private var targetVel = 2200.0
    private val velSlew = 7000.0
    private val powerSlewPerSec = 5.0
    var flyPower = 0.0
    private val maxVelEst = 4000.0
    private val kP_vel = 0.00032

    private val PIPELINE_INDEX = 2

    private var homing = false
    private val homePower = 0.50
    private val homeTol = 20
    private var turretIntegral = 0.0



    // ---------------------------------------------------------------------

    var dashboard: FtcDashboard = FtcDashboard.getInstance()

    var dashboardTelemetry: Telemetry = dashboard.telemetry

    // ---------------------------------------------------------------------


    var lastTime = System.nanoTime()


    fun startLimelight() {
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(PIPELINE_INDEX)
        limelight.start()
    }

   /*
   fun rotate(rotation: Int) {

        val target = servo.targetPosition + (1.0/3.0 * rotation)
        servo.targetPosition = (target % 1.0 + 1.0) % 1.0
        lastRotateTimeMs = System.currentTimeMillis()
    }
    */

    fun homeBiased(offset: Int = 5) {
        homing = true
        turretMotor.targetPosition = offset
        turretMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        turretMotor.power = homePower
    }

    fun home() {
        homeBiased(0)
    }

    fun homeLeft(offset: Int = -120) {
        homing = true
        turretMotor.targetPosition = offset
        turretMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        turretMotor.power = homePower
    }

    fun homeRight(offset: Int = 120) {
        homing = true
        turretMotor.targetPosition = offset
        turretMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        turretMotor.power = homePower
    }

   /* override fun update() {
        val now = System.nanoTime()
        val dt = (now - lastTime) / 1e9
        lastTime = now

        val result = limelight.getLatestResult()

        var launchDistVolts = launchDist.getVoltage()
        var distInLauncher = ((launchDistVolts / MaxVoltage) * MaxDistance_mm)

        var hasTargetNow = false
        var txDeg = 0.0
        var tyDeg = 0.0


        if (result != null && result.isValid) {
            val fid = result.fiducialResults

            val best: LLResultTypes.FiducialResult? =
                if (!fid.isNullOrEmpty()) {
                    if (aimTagId != null) {
                        fid.filter { it.fiducialId == aimTagId }.maxByOrNull { it.targetArea }
                    } else {
                        fid.maxByOrNull { it.targetArea }
                    }
                } else {
                    null
                }

            if (best != null) {
                hasTargetNow = true
                txDeg = best.targetXDegrees
                tyDeg = best.targetYDegrees
                lastTxDeg = txDeg
                lastTyDeg = tyDeg
                lastSeenTimeMs = System.currentTimeMillis()
            } else if (aimTagId == null) {
                hasTargetNow = true
                txDeg = result.tx
                tyDeg = result.ty
                lastTxDeg = txDeg
                lastTyDeg = tyDeg
                lastSeenTimeMs = System.currentTimeMillis()
            }


        }

        val nowMs = System.currentTimeMillis()
        val recentlySeen = (nowMs - lastSeenTimeMs) <= holdLastSeenMs



        if (homing) {
            val errHome = turretMotor.targetPosition - turretMotor.currentPosition
            if (abs(errHome) <= homeTol) {
                homing = false
                turretMotor.power = 0.0
                turretMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            } else {
                turretMotor.power = homePower
            }
        } else {
            val txToUse = when {
                hasTargetNow -> txDeg
                recentlySeen -> lastTxDeg
                else -> 0.0
            }


            if (abs(txToUse) > deadbandDeg) {
                turretIntegral += txToUse * dt
            } else {
                turretIntegral = 0.0
            }

            turretIntegral = clamp(turretIntegral, -200.0, 200.0)


            txVel = (txToUse - lastTx) / dt
            lastTx = txToUse
            val turretCmdPower =
                if (abs(txToUse) <= deadbandDeg) {
                    0.0
                } else {
                    var pwr = TurretConfig.kP_tt * txToUse - TurretConfig.kD_tt * txVel + TurretConfig.kI_tt * turretIntegral
                    if (abs(pwr) < minPower) pwr = 0.0
                    clamp(pwr, -maxPower_tt, maxPower_tt)
                }

            turretMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            turretMotor.power = turretCmdPower
        }

        if (hasTargetNow) {
            val angleRad = Math.toRadians(camMountDeg + tyDeg)
            val tanVal = tan(angleRad)
            if (tanVal > 0.01) {
                val d = (tagHeightCm - camHeightCm) / tanVal
                if (d.isFinite() && d > 0.0) {
                    distFiltCm = if ((nowMs - lastDistSeenMs) > holdDistMs || distFiltCm == 0.0) {
                        d
                    } else {
                        (1.0 - distAlpha) * distFiltCm + distAlpha * d
                    }
                    lastDistSeenMs = nowMs
                }
            }
        }

        val distFresh = (nowMs - lastDistSeenMs) <= holdDistMs

        val distPts = doubleArrayOf(80.0, 120.0, 160.0, 180.0, 200.0, 320.0)
        val velPts = doubleArrayOf(1690.0, 1870.6459, 1902.0, 2022.901, 2106.0, 2420.0)

        /*
     if (distFresh) {
            val v = interp1D(distFiltCm, distPts, velPts)
            targetVelCmd = v.coerceIn(minVel, maxVel)
        }

     */

        if (distFresh) {
            val v = interp1D(distFiltCm, distPts, velPts)


            if ((launchDistVolts < TurretConfig.spikeTriggerVolts) && !spiking && spindexer.wasRecentlyRotated()) {
                spiking = true
                spikeStartTime = System.currentTimeMillis()
            }
            if (spiking) {
                val elapsed = System.currentTimeMillis() - spikeStartTime
                if (elapsed >= TurretConfig.spikeDurationMs) {
                    spiking = false
                    targetVelCmd = v.coerceIn(minVel, maxVel)
                } else {
                    targetVelCmd = (v + TurretConfig.spikeVelIncrease).coerceIn(minVel, maxVel)
                }
            }
        }


        // ---------------------------------------------------------------------


        dashboardTelemetry.addData("dist volts", launchDistVolts)
        dashboardTelemetry.addData("Spiking", spiking)
        dashboardTelemetry.addData("Distance of Sensor in launcher", distInLauncher)
        dashboardTelemetry.addData("recentRotate", spindexer.wasRecentlyRotated())
        dashboardTelemetry.addData("lastRotateMs", spindexer.lastRotateTimeMs)
        dashboardTelemetry.update()

        // ---------------------------------------------------------------------

        targetVel = slew(targetVel, targetVelCmd, dt, velSlew)
        val measuredVel = abs(flywheelMotor.velocity)
        val ff = (targetVel / maxVelEst).coerceIn(0.0, 1.0)
        val err = targetVel - measuredVel
        val corr = kP_vel * err
        val out = (ff + corr + TurretConfig.FlywheelLowVoltageAdditive).coerceIn(0.0, 1.0)

        val RED = 0.28
        val YELLOW = 0.34
        val GREEN = 0.50
        val REDOVER = 0.28

        val error = targetVel - measuredVel

        val greenThreshold = 600.0  //green
        val yellowThreshold = 1450.0  //yellow
        val blinkThreshold = 2850.0   // red blink

        val blinkPeriod = 500L //.5 sec
        val blinkOn = (System.currentTimeMillis() / blinkPeriod) % 2 == 0L

        val colorPos = when {

            measuredVel + 500.0 > targetVel -> {
                if (blinkOn) REDOVER else 0.0
            }

            error > blinkThreshold -> {
                if (blinkOn) RED else 0.0
            }

            error > yellowThreshold -> RED
            error > greenThreshold -> YELLOW
            else -> GREEN
        }

        backlightR.setPosition(colorPos)
        backlightL.setPosition(colorPos)                                                                                                                                                                                                                                                                                                                                                      //chicken

        flyPower = rampTowards(flyPower, out, powerSlewPerSec * dt)

        if (shooting == true) {
            flywheelMotor.power = -flyPower
        } else if (shooting == false) {
            flywheelMotor.power = -0.4
        }

    }
}