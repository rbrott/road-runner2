package com.acmerobotics.roadrunner2

import kotlin.math.sqrt

class Time


fun clamp(x: Double, lo: Double, hi: Double): Double {
    if (x < lo) {
        return lo
    }
    if (x > hi) {
        return hi
    }
    return x
}


class Profile(val segments: List<Segment>) {
    interface Segment {
        val duration: Double
        operator fun get(t: Double): DualNum<Time>

        val distance: Double
        fun getByDisp(s: Double): DualNum<Time>

        fun end() = get(duration)
    }

    class ConstVelSegment(
        start: DualNum<Time>,
        val vel: Double,
        override val distance: Double) : Segment {
        override val duration = distance / vel

        val startPos = start.values[0]

        val n = start.values.size

        override fun get(t: Double): DualNum<Time> {
            val t = DualNum.variable<Time>(clamp(t, 0.0, duration), n)
            return startPos + vel * t
        }

        override fun getByDisp(s: Double) =
            DualNum<Time>(clamp(s, 0.0, distance), DualNum.constant(vel, n - 1))
    }

    class ConstAccelSegment(
        start: DualNum<Time>,
        val accel: Double,
        override val distance: Double) : Segment {
        override val duration = sqrt(2.0 * distance / accel)

        val startPos = start.values[0]
        val startVel = start.values[1]

        val n = start.values.size

        override fun get(t: Double): DualNum<Time> {
            val t = DualNum.variable<Time>(clamp(t, 0.0, duration), n)
            return startPos + startVel * t + 0.5 * accel * t.sqr()
        }

        override fun getByDisp(s: Double): DualNum<Time> {
            val s = clamp(s, 0.0, distance)
            return DualNum(s,
                DualNum(sqrt(startVel * startVel + 2.0 * accel * s),
                    DualNum.constant(accel, n - 2)))
        }
    }

    val duration = segments.sumOf { it.duration }
    operator fun get(t: Double): DualNum<Time> {
        var segmentTime = clamp(t, 0.0, duration)
        for (segment in segments) {
            if (segmentTime > segment.duration) {
                segmentTime -= segment.duration
                continue
            }
            return segment[segmentTime]
        }
        throw AssertionError()
    }

    val distance = segments.sumOf { it.distance }
    fun getByDisp(s: Double): DualNum<Time> {
        var segmentDisp = clamp(s, 0.0, distance)
        for (segment in segments) {
            if (segmentDisp > segment.distance) {
                segmentDisp -= segment.distance
                continue
            }
            return segment.getByDisp(segmentDisp)
        }
        throw AssertionError()
    }
}

fun genAccelLimitedProfile(distance: Double, maxVel: Double, maxAccel: Double, n: Int): Profile {
    if (maxVel <= 0.0) {
        throw IllegalArgumentException("max vel is negative")
    }
    if (maxAccel <= 0.0) {
        throw IllegalArgumentException("max accel is negative")
    }
    if (distance <= 0.0) {
        throw IllegalArgumentException("distance is negative")
    }

    val maxWedgeDistance = maxVel * maxVel / maxAccel
    return if (distance > maxWedgeDistance) {
        val accelSeg = Profile.ConstAccelSegment(DualNum.constant(0.0, n), maxAccel, maxVel / maxAccel)
        val coastSeg = Profile.ConstVelSegment(accelSeg.end(), maxVel, distance / maxVel - maxVel / maxAccel)
        val decelSeg = Profile.ConstAccelSegment(coastSeg.end(), -maxAccel, maxVel / maxAccel)
        Profile(listOf(accelSeg, coastSeg, decelSeg))
    } else {
        val segDuration = sqrt(distance / maxAccel)
        val accelSeg = Profile.ConstAccelSegment(DualNum.constant(0.0, n), maxAccel, segDuration)
        val decelSeg = Profile.ConstAccelSegment(accelSeg.end(), -maxAccel, segDuration)
        Profile(listOf(accelSeg, decelSeg))
    }
}

class Trajectory(private val path: TangentPath, private val profile: Profile) {
    val duration: Double = profile.duration

    operator fun <Robot> get(t: Double): Transform2<World, Robot, DualNum<Time>> = profile[t].let { s ->
        path.get<Robot>(s.values[0]).reparam(s) }

    fun <Robot> getByDisp(s: Double): Transform2<World, Robot, DualNum<Time>> = profile.getByDisp(s).let { s ->
        path.get<Robot>(s.values[0]).reparam(s) }

    fun <Robot> project(query: Position2<World, Robot, DualNum<Time>>, init: Double) =
        path.project(query.constant(), init).let { s ->
            val r = path.get<Robot>(s).origin()

            val d = query - r.reparam(profile.getByDisp(s))
            val drds = r.tangentVec().reparam(profile.getByDisp(s))
            val d2rds2 = r.tangentVec().drop(1).reparam(profile.getByDisp(s))

            val dsdt = (query.tangentVec() dot drds) / (1.0 - (d dot d2rds2))

            DualNum(s, dsdt)
        }
}
