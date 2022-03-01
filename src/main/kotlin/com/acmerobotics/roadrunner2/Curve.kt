package com.acmerobotics.roadrunner2

import kotlin.math.*

class Internal
class ArcLength

class World

private fun <A, B, C, D> approxLength(
    p1: Position2<A, B, DoubleNum>, p2: Position2<A, C, DoubleNum>, p3: Position2<A, D, DoubleNum>): DoubleNum {
    class Center

    val chord = p1 distTo p3

    val v1 = p2 - p1
    val v2 = p2 - p3
    val det = 4.0 * (v1 det v2)

    return if (det.value epsilonEquals 0.0) {
        chord
    } else {
        val origin = Position2.origin<A>()

        val x1 = (p1 - origin).sqrNorm()
        val x2 = (p2 - origin).sqrNorm()
        val x3 = (p3 - origin).sqrNorm()

        val y1 = x2 - x1
        val y2 = x2 - x3

        val center: Position2<A, Center, DoubleNum> = origin + Vector2(
            (y1 * v2.y - y2 * v1.y) / det, (y2 * v1.x - y1 * v2.x) / det)
        val radius = center distTo p1
        2.0 * radius * (chord / (2.0 * radius)).asin()
    }
}

interface PositionPath<Param> {
    val maxParam: Double
    operator fun <Robot> get(param: Double): Position2<World, Robot, DualNum<Param>>
}

fun <Robot> PositionPath<ArcLength>.project(query: Position2<World, Robot, DoubleNum>, init: Double): Double {
    return (1..10).fold(init) { s, _ ->
        class PathGuess
        val guess = get<PathGuess>(s)
        val ds = (query - guess.constant()) dot guess.tangentVec().constant()
        clamp(s + ds.value, 0.0, maxParam)
    }
}

class QuinticSpline1(
    start: Double,
    startDeriv: Double,
    startSecondDeriv: Double,
    end: Double,
    endDeriv: Double,
    endSecondDeriv: Double,
    private val n: Int
) {
    val a = DualNum.constant<Internal>(
        -6.0 * start - 3.0 * startDeriv - 0.5 * startSecondDeriv
        + 6.0 * end - 3.0 * endDeriv + 0.5 * endSecondDeriv,
        n
    )
    val b = DualNum.constant<Internal>(
        15.0 * start + 8.0 * startDeriv + 1.5 * startSecondDeriv
        - 15.0 * end + 7.0 * endDeriv - endSecondDeriv,
        n
    )
    val c = DualNum.constant<Internal>(
        -10.0 * start - 6.0 * startDeriv - 1.5 * startSecondDeriv
        + 10.0 * end - 4.0 * endDeriv + 0.5 * endSecondDeriv,
        n
    )
    val d = DualNum.constant<Internal>(0.5 * startSecondDeriv, n)
    val e = DualNum.constant<Internal>(startDeriv, n)
    val f = DualNum.constant<Internal>(start, n)

    operator fun get(t: Double): DualNum<Internal> {
        val t = DualNum.variable<Internal>(t, n)
        return ((((a * t + b) * t + c) * t + d) * t + e) * t + f
    }
}

class QuinticSpline2(
    private val x: QuinticSpline1,
    private val y: QuinticSpline1,
) : PositionPath<Internal> {
    override val maxParam = 1.0
    override fun <Robot> get(t: Double) = Position2<World, Robot, DualNum<Internal>>(x[t], y[t])
}

fun <A, B> Position2<A, B, DualNum<Internal>>.curvature(): Double {
    val (_, dx, d2x) = x.values
    val (_, dy, d2y) = y.values
    val derivNorm = sqrt(dx * dx + dy * dy)
    return abs(d2x * dy - dx * d2y) / (derivNorm * derivNorm * derivNorm)
}

private fun lerp(x: Double, fromLo: Double, fromHi: Double, toLo: Double, toHi: Double) =
    toLo + (x - fromLo) * (toHi - toLo) / (fromHi - fromLo)

class ArcApproxArcCurve2(
    private val curve: PositionPath<Internal>,
    private val maxDeltaK: Double = 0.01,
    private val maxSegmentLength: Double = 0.25,
    private val maxDepth: Int = 30,
) : PositionPath<ArcLength> {
    private data class Samples(
        val length: Double,
        // first: s, second: t
        val values: List<Pair<Double, Double>>,
    ) {
        init {
            if (values.size < 2) {
                throw IllegalArgumentException("must have at least two samples")
            }
        }

        operator fun plus(other: Samples) = Samples(
            length + other.length,
            values.dropLast(1) + other.values
        )
    }

    private fun adaptiveSample(): Samples {
        fun <RobotBegin, RobotEnd> helper(
            sLo: Double,
            tLo: Double,
            tHi: Double,
            pLo: Position2<World, RobotBegin, DualNum<Internal>>,
            pHi: Position2<World, RobotEnd, DualNum<Internal>>,
            depth: Int
        ): Samples {
            class RobotMid

            val tMid = 0.5 * (tLo + tHi)
            val pMid = curve.get<RobotMid>(tMid)

            val deltaK = abs(pLo.curvature() - pHi.curvature())
            val length = approxLength(pLo.constant(), pMid.constant(), pHi.constant()).value

            return if (depth < maxDepth && (deltaK > maxDeltaK || length > maxSegmentLength)) {
                val loSamples = helper(sLo, tLo, tMid, pLo, pMid, depth + 1)
                // loSamples.length is more accurate than length
                val sMid = sLo + loSamples.length
                val hiSamples = helper(sMid, tMid, tHi, pMid, pHi, depth + 1)
                loSamples + hiSamples
            } else {
                Samples(
                    length, listOf(
                        Pair(sLo, tLo),
                        Pair(sLo + length, tHi)
                    )
                )
            }
        }

        class RobotBegin
        class RobotEnd
        return helper<RobotBegin, RobotEnd>(0.0, 0.0, 1.0, curve[0.0], curve[1.0], 0)
    }

    override val maxParam: Double
    private val samples: List<Pair<Double, Double>>

    init {
        val (length, values) = adaptiveSample()
        this.maxParam = length
        this.samples = values
    }

    fun reparam(s: Double): Double {
        val result = samples.binarySearch { (sMid, _) -> sMid.compareTo(s) }
        return when {
            result >= 0 -> samples[result].let { (_, t) -> t }
            else -> {
                val insIndex = -(result + 1)
                when {
                    insIndex == 0 -> 0.0
                    insIndex >= samples.size -> 1.0
                    else -> {
                        val (sLo, tLo) = samples[insIndex - 1]
                        val (sHi, tHi) = samples[insIndex]
                        lerp(s, sLo, sHi, tLo, tHi)
                    }
                }
            }
        }
    }

    override fun <Robot> get(s: Double): Position2<World, Robot, DualNum<ArcLength>> {
        val t = reparam(s)
        val point = curve.get<Robot>(t)

        val tDerivs = point.free().drop(1).norm().recip()

        val dtds = tDerivs.values[0]
        val d2tds2 = tDerivs.reparam(DualNum<ArcLength>(listOf(t, dtds))).values[1]
        val d3tds3 = tDerivs.reparam(DualNum<ArcLength>(listOf(t, dtds, d2tds2))).values[2]

        return point.reparam(DualNum(listOf(t, dtds, d2tds2, d3tds3)))
    }
}

class TangentPath(private val curve: PositionPath<ArcLength>) {
    val length: Double = curve.maxParam

    operator fun <Robot> get(s: Double): Transform2<World, Robot, DualNum<ArcLength>> = curve.get<Robot>(s).let {
        Transform2.rotateThenTranslate(it.tangent(), it.free()) }

    fun <Robot> project(query: Position2<World, Robot, DoubleNum>, init: Double) = curve.project(query, init)
}
