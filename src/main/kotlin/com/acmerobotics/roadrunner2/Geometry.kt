package com.acmerobotics.roadrunner2

data class Position2<Origin, Point, N : Num<N>>(val x: N, val y: N) {
    companion object {
        fun <Origin, Point, Param> constant(x: Double, y: Double, n: Int) = Position2<Origin, Point, DualNum<Param>>(
            DualNum.constant(x, n), DualNum.constant(y, n))

        fun <Origin> origin() = Position2<Origin, Origin, DoubleNum>(DoubleNum(0.0), DoubleNum(0.0))
        fun <Origin, Param> origin(n : Int) = Position2<Origin, Origin, DualNum<Param>>(DualNum.constant(0.0, n), DualNum.constant(0.0, n))

        fun <Origin, Point, N : Num<N>> bind(v: Vector2<N>) = Position2<Origin, Point, N>(v.x, v.y)
    }

    operator fun <End> minus(other: Position2<Origin, End, N>) = Vector2(x - other.x, y - other.y)
    operator fun <End> plus(diff: Vector2<N>) = Position2<Origin, End, N>(x + diff.x, y + diff.y)

    infix fun <OtherPoint> distTo(other: Position2<Origin, OtherPoint, N>) = (this - other).norm()

    fun free() = Vector2(x, y)
}

fun <Origin, Point, Param, NewParam> Position2<Origin, Point, DualNum<Param>>.reparam(oldParam: DualNum<NewParam>) =
    Position2<Origin, Point, DualNum<NewParam>>(x.reparam(oldParam), y.reparam(oldParam))

fun <Origin, Point, Param> Position2<Origin, Point, DualNum<Param>>.tangent() = Rotation2(x.drop(1), y.drop(1))
fun <Origin, Point, Param> Position2<Origin, Point, DualNum<Param>>.tangentVec() = Vector2(x.drop(1), y.drop(1))
fun <Origin, Point, Param> Position2<Origin, Point, DualNum<Param>>.take(n: Int) = Position2<Origin, Point, DualNum<Param>>(x.take(n), y.take(n))
fun <Origin, Point, Param> Position2<Origin, Point, DualNum<Param>>.constant() = Position2<Origin, Point, DoubleNum>(x.constant(), y.constant())

data class Vector2<N : Num<N>>(val x: N, val y: N) {
    companion object {
        fun <Param> constant(x: Double, y: Double, n: Int) = Vector2<DualNum<Param>>(DualNum.constant(x, n), DualNum.constant(y, n))
    }

    operator fun plus(other: Vector2<N>) = Vector2(x + other.x, y + other.y)

    operator fun times(scalar: Double) = Vector2(scalar * x, scalar * y)

    operator fun unaryMinus() = Vector2(-x, -y)

    infix fun dot(other: Vector2<N>) = x * other.x + y * other.y
    fun sqrNorm() = this dot this
    fun norm() = sqrNorm().sqrt()

    infix fun det(other: Vector2<N>) = x * other.y - y * other.x
}

fun <Param> Vector2<DualNum<Param>>.take(n: Int) = Vector2(x.take(n), y.take(n))

fun <Param> Vector2<DualNum<Param>>.drop(n: Int) = Vector2(x.drop(n), y.drop(n))

fun <Param, NewParam> Vector2<DualNum<Param>>.reparam(oldParam: DualNum<NewParam>) =
    Vector2(x.reparam(oldParam), y.reparam(oldParam))

fun <Param> Vector2<DualNum<Param>>.constant() = Vector2(x.constant(), y.constant())

operator fun <N : Num<N>> Double.times(other: Vector2<N>) = other * this

class Rotation2<N : Num<N>>(real: N, imag: N) {
    val real: N
    val imag: N

    companion object {
        fun <N : Num<N>> exp(theta: N) = Rotation2(theta.cos(), theta.sin())
    }

    init {
        val norm = (real * real + imag * imag).sqrt()
        this.real = real / norm
        this.imag = imag / norm
    }

    operator fun times(vector: Vector2<N>) = Vector2(
        real * vector.x - imag * vector.y,
        imag * vector.x + real * vector.y
    )

    operator fun times(other: Rotation2<N>) = Rotation2(
        real * other.real - imag * other.imag,
        real * other.imag + imag * other.real
    )

    operator fun <Dst2, Dst, Src> times(other: Transform2<Dst, Src, N>): Transform2<Dst2, Src, N> =
        Transform2.rotateThenTranslate(this * other.rotation, this * other.translation)

    fun inverse() = Rotation2(real, -imag)

    fun log() = (imag / real).atan()

    override fun toString() = "Rotation2(real=$real, imag=$imag)"
}

fun <Param> Rotation2<DualNum<Param>>.constant() = Rotation2(real.constant(), imag.constant())

fun <OldParam, NewParam> Rotation2<DualNum<OldParam>>.reparam(oldParam: DualNum<NewParam>) =
    Rotation2(real.reparam(oldParam), imag.reparam(oldParam))

class Transform2<Dst, Src, N : Num<N>> private constructor(
    val rotation: Rotation2<N>,
    val translation: Vector2<N>
) {
    companion object {
        fun <Dst, Src, N : Num<N>> rotateThenTranslate(
            rotation: Rotation2<N>,
            translation: Vector2<N>
        ) = Transform2<Dst, Src, N>(rotation, translation)

        // see (133), (134) in https://ethaneade.com/lie.pdf
        private fun <N : Num<N>> entries(theta: N) : Pair<N, N> {
            val A = if (theta.value() epsilonEquals 0.0) {
                1.0 - theta * theta / 6.0
            } else {
                theta.sin() / theta
            }
            val B = if (theta.value() epsilonEquals 0.0) {
                theta / 2.0
            } else {
                (1.0 - theta.cos()) / theta
            }
            return Pair(A, B)
        }

        fun <Dst, Src, N : Num<N>> exp(twist: Twist2<N>) : Transform2<Dst, Src, N> {
            val (x, y, theta) = twist

            val rotation = Rotation2.exp(theta)

            val (A, B) = entries(theta)
            val translation = Vector2(
                A * x - B * y,
                B * x + A * y
            )

            return rotateThenTranslate(rotation, translation)
        }
    }

    fun origin() = Position2.bind<Dst, Src, N>(translation)

    operator fun times(other: Vector2<N>) = rotation * other

    operator fun <Src2> times(other: Transform2<Src, Src2, N>): Transform2<Dst, Src2, N> =
        rotateThenTranslate(rotation * other.rotation, rotation * other.translation + translation)

    fun inverse(): Transform2<Src, Dst, N> = rotateThenTranslate(rotation.inverse(), rotation.inverse() * -translation)

    fun log(): Twist2<N> {
        val theta = rotation.log()

        val (A, B) = entries(theta)
        val denom = Vector2(A, B).sqrNorm()

        val (x, y) = translation
        return Twist2(
            (A * x + B * y) / denom,
            (-B * x + A * y) / denom,
            theta
        )
    }

    operator fun <Src2> minus(other: Transform2<Dst, Src2, N>) = (other.inverse() * this).log()
    operator fun <Src2> plus(other: Twist2<N>) = this * exp<Src, Src2, N>(other)

    override fun toString() = "Transform2(rotation=$rotation, translation=$translation)"
}

fun <Dst, Src, Param> Transform2<Dst, Src, DualNum<Param>>.constant() =
    Transform2.rotateThenTranslate<Dst, Src, DoubleNum>(rotation.constant(), translation.constant())

fun <Dst, Src, OldParam, NewParam> Transform2<Dst, Src, DualNum<OldParam>>.reparam(oldParam: DualNum<NewParam>) =
        Transform2.rotateThenTranslate<Dst, Src, DualNum<NewParam>>(rotation.reparam(oldParam), translation.reparam(oldParam))

data class Twist2<N : Num<N>>(val x: N, val y: N, val theta: N)
