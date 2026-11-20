// KartSim core math. Double precision, ISO 8855 axes (x fwd, y left, z up), SI units.
// Deliberately independent of Unreal Engine types so the simulation compiles standalone.
#pragma once

#include "KartSim/Export.h"

#include <cmath>

namespace kartsim
{
	constexpr double kPi = 3.14159265358979323846;
	constexpr double kGravity = 9.80665;
	constexpr double kRpmToRadS = 2.0 * kPi / 60.0;
	constexpr double kRadSToRpm = 60.0 / (2.0 * kPi);
	constexpr double kDegToRad = kPi / 180.0;
	constexpr double kRadToDeg = 180.0 / kPi;

	inline double Clamp(double V, double Lo, double Hi) { return V < Lo ? Lo : (V > Hi ? Hi : V); }
	inline double Sign(double V) { return V > 0.0 ? 1.0 : (V < 0.0 ? -1.0 : 0.0); }
	inline double Lerp(double A, double B, double T) { return A + (B - A) * T; }
	inline double Smoothstep(double T)
	{
		T = Clamp(T, 0.0, 1.0);
		return T * T * (3.0 - 2.0 * T);
	}
	inline double Sq(double V) { return V * V; }

	struct Vec3
	{
		double X = 0.0, Y = 0.0, Z = 0.0;

		constexpr Vec3() = default;
		constexpr Vec3(double InX, double InY, double InZ) : X(InX), Y(InY), Z(InZ) {}

		Vec3 operator+(const Vec3& O) const { return {X + O.X, Y + O.Y, Z + O.Z}; }
		Vec3 operator-(const Vec3& O) const { return {X - O.X, Y - O.Y, Z - O.Z}; }
		Vec3 operator-() const { return {-X, -Y, -Z}; }
		Vec3 operator*(double S) const { return {X * S, Y * S, Z * S}; }
		Vec3 operator/(double S) const { return {X / S, Y / S, Z / S}; }
		Vec3& operator+=(const Vec3& O) { X += O.X; Y += O.Y; Z += O.Z; return *this; }
		Vec3& operator-=(const Vec3& O) { X -= O.X; Y -= O.Y; Z -= O.Z; return *this; }
		Vec3& operator*=(double S) { X *= S; Y *= S; Z *= S; return *this; }

		double Dot(const Vec3& O) const { return X * O.X + Y * O.Y + Z * O.Z; }
		Vec3 Cross(const Vec3& O) const { return {Y * O.Z - Z * O.Y, Z * O.X - X * O.Z, X * O.Y - Y * O.X}; }
		double LengthSq() const { return Dot(*this); }
		double Length() const { return std::sqrt(LengthSq()); }
		Vec3 Normalized() const
		{
			const double L = Length();
			return L > 1e-12 ? (*this) / L : Vec3(0.0, 0.0, 0.0);
		}
		Vec3 Mul(const Vec3& O) const { return {X * O.X, Y * O.Y, Z * O.Z}; }
	};
	inline Vec3 operator*(double S, const Vec3& V) { return V * S; }

	// Unit quaternion, Hamilton convention, rotates body -> world.
	struct Quat
	{
		double W = 1.0, X = 0.0, Y = 0.0, Z = 0.0;

		constexpr Quat() = default;
		constexpr Quat(double InW, double InX, double InY, double InZ) : W(InW), X(InX), Y(InY), Z(InZ) {}

		static Quat FromAxisAngle(const Vec3& Axis, double Angle)
		{
			const Vec3 A = Axis.Normalized();
			const double S = std::sin(0.5 * Angle);
			return {std::cos(0.5 * Angle), A.X * S, A.Y * S, A.Z * S};
		}

		Quat operator*(const Quat& O) const
		{
			return {W * O.W - X * O.X - Y * O.Y - Z * O.Z,
			        W * O.X + X * O.W + Y * O.Z - Z * O.Y,
			        W * O.Y - X * O.Z + Y * O.W + Z * O.X,
			        W * O.Z + X * O.Y - Y * O.X + Z * O.W};
		}
		Quat Conjugate() const { return {W, -X, -Y, -Z}; }
		double Dot(const Quat& O) const { return W * O.W + X * O.X + Y * O.Y + Z * O.Z; }

		Quat Normalized() const
		{
			const double L = std::sqrt(W * W + X * X + Y * Y + Z * Z);
			return L > 1e-12 ? Quat(W / L, X / L, Y / L, Z / L) : Quat();
		}

		Vec3 Rotate(const Vec3& V) const
		{
			// v' = v + 2w(q x v) + 2 q x (q x v)
			const Vec3 Q(X, Y, Z);
			const Vec3 T = Q.Cross(V) * 2.0;
			return V + T * W + Q.Cross(T);
		}
		Vec3 Unrotate(const Vec3& V) const { return Conjugate().Rotate(V); }

		// Integrate with body-frame angular velocity over dt (exact for constant omega).
		Quat IntegrateBody(const Vec3& OmegaBody, double Dt) const
		{
			const double Angle = OmegaBody.Length() * Dt;
			if (Angle < 1e-12)
			{
				return *this;
			}
			return ((*this) * FromAxisAngle(OmegaBody, Angle)).Normalized();
		}

		static Quat Slerp(const Quat& A, Quat B, double T)
		{
			double D = A.Dot(B);
			if (D < 0.0)
			{
				B = Quat(-B.W, -B.X, -B.Y, -B.Z);
				D = -D;
			}
			if (D > 0.9995)
			{
				return Quat(Lerp(A.W, B.W, T), Lerp(A.X, B.X, T), Lerp(A.Y, B.Y, T), Lerp(A.Z, B.Z, T)).Normalized();
			}
			const double Theta = std::acos(D);
			const double S = std::sin(Theta);
			const double Wa = std::sin((1.0 - T) * Theta) / S;
			const double Wb = std::sin(T * Theta) / S;
			return Quat(A.W * Wa + B.W * Wb, A.X * Wa + B.X * Wb, A.Y * Wa + B.Y * Wb, A.Z * Wa + B.Z * Wb);
		}

		// Yaw (about z), pitch (about y), roll (about x), ZYX intrinsic. Radians.
		void ToEuler(double& OutRoll, double& OutPitch, double& OutYaw) const
		{
			OutRoll = std::atan2(2.0 * (W * X + Y * Z), 1.0 - 2.0 * (X * X + Y * Y));
			OutPitch = std::asin(Clamp(2.0 * (W * Y - Z * X), -1.0, 1.0));
			OutYaw = std::atan2(2.0 * (W * Z + X * Y), 1.0 - 2.0 * (Y * Y + Z * Z));
		}
	};

	// Rodrigues rotation of V about unit axis K by angle A.
	inline Vec3 RotateAboutAxis(const Vec3& V, const Vec3& K, double A)
	{
		const double C = std::cos(A), S = std::sin(A);
		return V * C + K.Cross(V) * S + K * (K.Dot(V) * (1.0 - C));
	}

	// Piecewise-linear lookup table (x must be ascending). Clamps at the ends.
	struct Curve1D
	{
		static constexpr int MaxPoints = 64;
		double Xs[MaxPoints] = {};
		double Ys[MaxPoints] = {};
		int Count = 0;

		void Add(double X, double Y)
		{
			if (Count < MaxPoints)
			{
				Xs[Count] = X;
				Ys[Count] = Y;
				++Count;
			}
		}
		double Eval(double X) const
		{
			if (Count == 0) { return 0.0; }
			if (X <= Xs[0]) { return Ys[0]; }
			if (X >= Xs[Count - 1]) { return Ys[Count - 1]; }
			int Lo = 0, Hi = Count - 1;
			while (Hi - Lo > 1)
			{
				const int Mid = (Lo + Hi) / 2;
				if (Xs[Mid] <= X) { Lo = Mid; } else { Hi = Mid; }
			}
			const double T = (X - Xs[Lo]) / (Xs[Hi] - Xs[Lo]);
			return Lerp(Ys[Lo], Ys[Hi], T);
		}
	};
}
