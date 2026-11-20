// Tester licensing: SHA-256 against FIPS test vectors, RSA verification against keys made by the Python key tool
// (Tests/LicenseTestVectors.h, a throwaway key pair), machine code format.
#include "TestFramework.h"

#include "KartSim/License.h"
#include "LicenseTestVectors.h"

#include <chrono>
#include <string>

using namespace kartsim::license;

namespace
{
	std::vector<uint8_t> HexBytes(const char* H)
	{
		std::vector<uint8_t> V;
		auto Nib = [](char C) { return C >= 'a' ? C - 'a' + 10 : C - '0'; };
		for (size_t I = 0; H[I] && H[I + 1]; I += 2) { V.push_back(uint8_t(Nib(H[I]) * 16 + Nib(H[I + 1]))); }
		return V;
	}
}

KS_TEST(License_Sha256Vectors)
{
	KS_EXPECT(Sha256Hex("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
	KS_EXPECT(Sha256Hex("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
	KS_EXPECT(Sha256Hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq") == "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
	KS_EXPECT(Sha256Hex(std::string(1000000, 'a')) == "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

KS_TEST(License_KeysFromTheToolVerify)
{
	const std::vector<uint8_t> N = HexBytes(LtModulusHex);
	const std::string M = "ABCD-EFGH-JKLM-NPQR";
	const auto T0 = std::chrono::steady_clock::now();
	const LicenseInfo Good = CheckLicense(LtGoodKey, N, 65537u, M, 20261201);
	const double Ms = 1000.0 * std::chrono::duration<double>(std::chrono::steady_clock::now() - T0).count();
	KS_LOG("good key: valid %d (%s), name '%s', expires %d, check %.1f ms", Good.bValid ? 1 : 0, Good.Error.c_str(), Good.Name.c_str(), Good.Expires, Ms);
	KS_EXPECT(Good.bValid && Good.Name == "Test Tester" && Good.Expires == 20261220 && Good.KeyId == "0badc0de");
	const LicenseInfo OtherPc = CheckLicense(LtGoodKey, N, 65537u, "ZZZZ-ZZZZ-ZZZZ-ZZZZ", 20261201);
	const LicenseInfo Expired = CheckLicense(LtGoodKey, N, 65537u, M, 20261221);
	const LicenseInfo Forged = CheckLicense(LtForgedKey, N, 65537u, M, 20261201);
	const LicenseInfo Tampered = CheckLicense(LtTamperedKey, N, 65537u, M, 20261201);
	const LicenseInfo Junk = CheckLicense("hello.world", N, 65537u, M, 20261201);
	const LicenseInfo Empty = CheckLicense("", N, 65537u, M, 20261201);
	KS_LOG("other PC: %s | expired: %s | forged: %s | tampered: %s | junk: %s", OtherPc.Error.c_str(), Expired.Error.c_str(), Forged.Error.c_str(),
	       Tampered.Error.c_str(), Junk.Error.c_str());
	KS_EXPECT(!OtherPc.bValid && !Expired.bValid && !Forged.bValid && !Tampered.bValid && !Junk.bValid && !Empty.bValid);
	// A key pasted with line breaks / spaces (e-mail clients wrap) still works.
	std::string Wrapped = LtGoodKey;
	Wrapped.insert(40, "\r\n ");
	KS_EXPECT(CheckLicense(Wrapped, N, 65537u, M, 20261201).bValid);
}

KS_TEST(License_MachineCodeFormat)
{
	const std::string A = MachineCodeFrom("guid-1|serial-1"), B = MachineCodeFrom("guid-2|serial-1");
	KS_LOG("machine codes: %s / %s", A.c_str(), B.c_str());
	KS_EXPECT(A.size() == 19 && A[4] == '-' && A[9] == '-' && A[14] == '-' && A != B && A == MachineCodeFrom("guid-1|serial-1"));
	for (char C : A) { KS_EXPECT(C == '-' || (C >= 'A' && C <= 'Z' && C != 'I' && C != 'O') || (C >= '2' && C <= '9')); }
}

KS_TEST(License_RevocationListsFromTheTool)
{
	const std::vector<uint8_t> N = HexBytes(LtModulusHex);
	const LicenseInfo Good = CheckLicense(LtGoodKey, N, 65537u, "ABCD-EFGH-JKLM-NPQR", 20261201);
	const RevocationList ById = ParseRevocationList(LtRevokesGoodById, N, 65537u);
	const RevocationList ByMachine = ParseRevocationList(LtRevokesGoodByMachine, N, 65537u);
	const RevocationList Others = ParseRevocationList(LtRevokesOthers, N, 65537u);
	const RevocationList Forged = ParseRevocationList(LtRevokesForged, N, 65537u);
	KS_LOG("lists: by id valid %d serial %lld (%zu ids) | by machine valid %d | others valid %d | forged valid %d", ById.bValid ? 1 : 0, ById.Serial, ById.KeyIds.size(),
	       ByMachine.bValid ? 1 : 0, Others.bValid ? 1 : 0, Forged.bValid ? 1 : 0);
	KS_EXPECT(Good.bValid);
	KS_EXPECT(ById.bValid && ById.Serial == 5 && ById.Date == 20261120 && IsRevoked(ById, Good));
	KS_EXPECT(ByMachine.bValid && IsRevoked(ByMachine, Good));
	KS_EXPECT(Others.bValid && !IsRevoked(Others, Good));
	KS_EXPECT(!Forged.bValid && !IsRevoked(Forged, Good)); // a list not signed by the developer is ignored
	KS_EXPECT(!ParseRevocationList("garbage", N, 65537u).bValid);
}
