// Minimal dependency-free test framework for the KartSim core.
#pragma once

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace kstest
{
	struct TestCase
	{
		const char* Name;
		std::function<void()> Fn;
	};

	inline std::vector<TestCase>& Registry()
	{
		static std::vector<TestCase> R;
		return R;
	}

	inline int& FailCount()
	{
		static int N = 0;
		return N;
	}

	inline bool& CurrentFailed()
	{
		static bool B = false;
		return B;
	}

	struct Registrar
	{
		Registrar(const char* Name, std::function<void()> Fn) { Registry().push_back({Name, std::move(Fn)}); }
	};

	inline void ReportFailure(const char* File, int Line, const std::string& Msg)
	{
		std::printf("    FAIL %s:%d: %s\n", File, Line, Msg.c_str());
		CurrentFailed() = true;
	}

	inline int RunAll(const char* Filter)
	{
		int Passed = 0, Failed = 0;
		for (const TestCase& T : Registry())
		{
			if (Filter && std::string(T.Name).find(Filter) == std::string::npos)
			{
				continue;
			}
			CurrentFailed() = false;
			std::printf("[ RUN  ] %s\n", T.Name);
			T.Fn();
			if (CurrentFailed()) { ++Failed; std::printf("[ FAIL ] %s\n", T.Name); }
			else { ++Passed; std::printf("[  OK  ] %s\n", T.Name); }
		}
		std::printf("\n%d passed, %d failed\n", Passed, Failed);
		return Failed;
	}
}

#define KS_CONCAT2(A, B) A##B
#define KS_CONCAT(A, B) KS_CONCAT2(A, B)
#define KS_TEST(Name) \
	static void Name(); \
	static kstest::Registrar KS_CONCAT(Reg_, Name)(#Name, &Name); \
	static void Name()

#define KS_EXPECT(Cond) \
	do { if (!(Cond)) { kstest::ReportFailure(__FILE__, __LINE__, "expected: " #Cond); } } while (0)

#define KS_EXPECT_NEAR(A, B, Tol) \
	do { const double KsA = (A), KsB = (B); if (!(std::fabs(KsA - KsB) <= (Tol))) { \
		char KsBuf[256]; std::snprintf(KsBuf, sizeof(KsBuf), "%s = %.6g, expected %.6g +/- %.3g", #A, KsA, KsB, (double)(Tol)); \
		kstest::ReportFailure(__FILE__, __LINE__, KsBuf); } } while (0)

#define KS_LOG(...) do { std::printf("    "); std::printf(__VA_ARGS__); std::printf("\n"); } while (0)
