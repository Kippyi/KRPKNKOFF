#include "KartSim/License.h"

#include <algorithm>
#include <cstring>

namespace kartsim
{
	namespace license
	{
		namespace
		{
			// ---- SHA-256 (FIPS 180-4) ----
			constexpr uint32_t Kc[64] = {
				0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
				0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
				0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
				0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
				0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
				0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
				0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
				0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
			uint32_t Rotr(uint32_t X, int N) { return (X >> N) | (X << (32 - N)); }

			void Block(uint32_t H[8], const uint8_t* P)
			{
				uint32_t W[64];
				for (int I = 0; I < 16; ++I) { W[I] = (uint32_t(P[4 * I]) << 24) | (uint32_t(P[4 * I + 1]) << 16) | (uint32_t(P[4 * I + 2]) << 8) | uint32_t(P[4 * I + 3]); }
				for (int I = 16; I < 64; ++I)
				{
					const uint32_t S0 = Rotr(W[I - 15], 7) ^ Rotr(W[I - 15], 18) ^ (W[I - 15] >> 3);
					const uint32_t S1 = Rotr(W[I - 2], 17) ^ Rotr(W[I - 2], 19) ^ (W[I - 2] >> 10);
					W[I] = W[I - 16] + S0 + W[I - 7] + S1;
				}
				uint32_t A = H[0], B = H[1], C = H[2], D = H[3], E = H[4], F = H[5], G = H[6], Hh = H[7];
				for (int I = 0; I < 64; ++I)
				{
					const uint32_t S1 = Rotr(E, 6) ^ Rotr(E, 11) ^ Rotr(E, 25);
					const uint32_t Ch = (E & F) ^ (~E & G);
					const uint32_t T1 = Hh + S1 + Ch + Kc[I] + W[I];
					const uint32_t S0 = Rotr(A, 2) ^ Rotr(A, 13) ^ Rotr(A, 22);
					const uint32_t Mj = (A & B) ^ (A & C) ^ (B & C);
					const uint32_t T2 = S0 + Mj;
					Hh = G; G = F; F = E; E = D + T1; D = C; C = B; B = A; A = T1 + T2;
				}
				H[0] += A; H[1] += B; H[2] += C; H[3] += D; H[4] += E; H[5] += F; H[6] += G; H[7] += Hh;
			}

			// ---- big unsigned integers, 32-bit limbs, little-endian ----
			using Big = std::vector<uint32_t>;
			Big FromBytesBE(const std::vector<uint8_t>& B, size_t Limbs)
			{
				Big R(Limbs, 0u);
				for (size_t I = 0; I < B.size(); ++I)
				{
					const size_t Bit = (B.size() - 1 - I) * 8;
					if (Bit / 32 < Limbs) { R[Bit / 32] |= uint32_t(B[I]) << (Bit % 32); }
				}
				return R;
			}
			std::vector<uint8_t> ToBytesBE(const Big& A, size_t Bytes)
			{
				std::vector<uint8_t> R(Bytes, 0);
				for (size_t I = 0; I < Bytes; ++I)
				{
					const size_t Bit = (Bytes - 1 - I) * 8;
					R[I] = Bit / 32 < A.size() ? uint8_t(A[Bit / 32] >> (Bit % 32)) : 0;
				}
				return R;
			}
			int Cmp(const Big& A, const Big& B) // same length
			{
				for (size_t I = A.size(); I-- > 0;) { if (A[I] != B[I]) { return A[I] < B[I] ? -1 : 1; } }
				return 0;
			}
			void SubInPlace(Big& A, const Big& B) // A -= B, A >= B, same length
			{
				uint64_t Borrow = 0;
				for (size_t I = 0; I < A.size(); ++I)
				{
					const uint64_t D = uint64_t(A[I]) - B[I] - Borrow;
					A[I] = uint32_t(D);
					Borrow = (D >> 63) & 1u;
				}
			}
			// (A * B) mod N; A, B < N, all N.size() limbs. Schoolbook product, bitwise long-division remainder
			// (only used a few dozen times per license check).
			Big MulMod(const Big& A, const Big& B, const Big& N)
			{
				const size_t L = N.size();
				Big P(2 * L, 0u);
				for (size_t I = 0; I < L; ++I)
				{
					uint64_t Carry = 0;
					for (size_t J = 0; J < L; ++J)
					{
						const uint64_t T = uint64_t(A[I]) * B[J] + P[I + J] + Carry;
						P[I + J] = uint32_t(T);
						Carry = T >> 32;
					}
					P[I + L] = uint32_t(Carry);
				}
				Big R(L + 1, 0u), Nx(N);
				Nx.push_back(0u);
				for (size_t Bit = 2 * L * 32; Bit-- > 0;)
				{
					// R = (R << 1) | bit
					uint32_t Carry = (P[Bit / 32] >> (Bit % 32)) & 1u;
					for (size_t K = 0; K < R.size(); ++K) { const uint32_t Nc = R[K] >> 31; R[K] = (R[K] << 1) | Carry; Carry = Nc; }
					if (Cmp(R, Nx) >= 0) { SubInPlace(R, Nx); }
				}
				R.pop_back();
				return R;
			}

			int B64Val(char C)
			{
				if (C >= 'A' && C <= 'Z') { return C - 'A'; }
				if (C >= 'a' && C <= 'z') { return C - 'a' + 26; }
				if (C >= '0' && C <= '9') { return C - '0' + 52; }
				if (C == '-' || C == '+') { return 62; }
				if (C == '_' || C == '/') { return 63; }
				return -1;
			}

			int ParseYmd(const std::string& S)
			{
				if (S.size() != 8 || !std::all_of(S.begin(), S.end(), [](char C) { return C >= '0' && C <= '9'; })) { return 0; }
				return std::atoi(S.c_str());
			}
		}

		void Sha256(const uint8_t* Data, size_t Len, uint8_t Out[32])
		{
			uint32_t H[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
			size_t I = 0;
			for (; I + 64 <= Len; I += 64) { Block(H, Data + I); }
			uint8_t Tail[128] = {};
			const size_t Rem = Len - I;
			std::memcpy(Tail, Data + I, Rem);
			Tail[Rem] = 0x80;
			const size_t TailLen = Rem + 9 <= 64 ? 64 : 128;
			const uint64_t Bits = uint64_t(Len) * 8;
			for (int K = 0; K < 8; ++K) { Tail[TailLen - 1 - K] = uint8_t(Bits >> (8 * K)); }
			Block(H, Tail);
			if (TailLen == 128) { Block(H, Tail + 64); }
			for (int K = 0; K < 8; ++K) { Out[4 * K] = uint8_t(H[K] >> 24); Out[4 * K + 1] = uint8_t(H[K] >> 16); Out[4 * K + 2] = uint8_t(H[K] >> 8); Out[4 * K + 3] = uint8_t(H[K]); }
		}

		std::string Sha256Hex(const std::string& Text)
		{
			uint8_t D[32];
			Sha256(reinterpret_cast<const uint8_t*>(Text.data()), Text.size(), D);
			static const char* Hex = "0123456789abcdef";
			std::string S;
			for (uint8_t B : D) { S += Hex[B >> 4]; S += Hex[B & 15]; }
			return S;
		}

		bool Base64UrlDecode(const std::string& In, std::vector<uint8_t>& Out)
		{
			Out.clear();
			uint32_t Acc = 0;
			int Bits = 0;
			for (char C : In)
			{
				if (C == '=' || C == ' ' || C == '\r' || C == '\n' || C == '\t') { continue; }
				const int V = B64Val(C);
				if (V < 0) { return false; }
				Acc = (Acc << 6) | uint32_t(V);
				Bits += 6;
				if (Bits >= 8) { Bits -= 8; Out.push_back(uint8_t(Acc >> Bits)); }
			}
			return true;
		}

		bool RsaVerifySha256(const std::vector<uint8_t>& ModulusBE, uint32_t E, const std::string& Message, const std::vector<uint8_t>& Signature)
		{
			if (E != 65537u || ModulusBE.size() < 128 || Signature.size() != ModulusBE.size()) { return false; }
			const size_t K = ModulusBE.size();
			const size_t Limbs = (K + 3) / 4;
			const Big N = FromBytesBE(ModulusBE, Limbs);
			const Big S = FromBytesBE(Signature, Limbs);
			if (Cmp(S, N) >= 0) { return false; }
			Big R = S;
			for (int I = 0; I < 16; ++I) { R = MulMod(R, R, N); } // S^(2^16)
			R = MulMod(R, S, N);                                    // S^65537
			const std::vector<uint8_t> Em = ToBytesBE(R, K);
			// EM = 00 01 FF..FF 00 || DigestInfo(SHA-256) || H
			static const uint8_t Prefix[19] = {0x30, 0x31, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86, 0x48, 0x01, 0x65, 0x03, 0x04, 0x02, 0x01, 0x05, 0x00, 0x04, 0x20};
			uint8_t H[32];
			Sha256(reinterpret_cast<const uint8_t*>(Message.data()), Message.size(), H);
			const size_t TLen = 19 + 32;
			if (K < TLen + 11) { return false; }
			std::vector<uint8_t> Want(K, 0xFF);
			Want[0] = 0x00;
			Want[1] = 0x01;
			Want[K - TLen - 1] = 0x00;
			std::memcpy(&Want[K - TLen], Prefix, 19);
			std::memcpy(&Want[K - 32], H, 32);
			uint8_t Diff = 0;
			for (size_t I = 0; I < K; ++I) { Diff |= uint8_t(Em[I] ^ Want[I]); }
			return Diff == 0;
		}

		std::string MachineCodeFrom(const std::string& Identity)
		{
			uint8_t D[32];
			const std::string Salted = "KRPK-MACHINE|" + Identity;
			Sha256(reinterpret_cast<const uint8_t*>(Salted.data()), Salted.size(), D);
			static const char* Alpha = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789"; // no I / O / 0 / 1 (read aloud / typed)
			std::string Code;
			uint64_t Acc = 0;
			int Bits = 0, Byte = 0;
			while (Code.size() < 19)
			{
				if (Bits < 5) { Acc = (Acc << 8) | D[Byte++]; Bits += 8; }
				Bits -= 5;
				Code += Alpha[(Acc >> Bits) & 31u];
				if (Code.size() == 4 || Code.size() == 9 || Code.size() == 14) { Code += '-'; }
			}
			return Code;
		}

		LicenseInfo CheckLicense(const std::string& KeyIn, const std::vector<uint8_t>& ModulusBE, uint32_t E, const std::string& MachineCode, int TodayYmd)
		{
			LicenseInfo L;
			std::string Key;
			for (char C : KeyIn) { if (C != ' ' && C != '\r' && C != '\n' && C != '\t') { Key += C; } }
			const size_t Dot = Key.find('.');
			if (Key.empty()) { L.Error = "No license key entered."; return L; }
			if (Dot == std::string::npos) { L.Error = "That is not a license key (copy the whole key)."; return L; }
			std::vector<uint8_t> PayloadB, Sig;
			if (!Base64UrlDecode(Key.substr(0, Dot), PayloadB) || !Base64UrlDecode(Key.substr(Dot + 1), Sig)) { L.Error = "The key is damaged (copy the whole key)."; return L; }
			const std::string Payload(PayloadB.begin(), PayloadB.end());
			if (!RsaVerifySha256(ModulusBE, E, Payload, Sig)) { L.Error = "This key was not issued for this game, or it was changed."; return L; }
			std::vector<std::string> F;
			size_t Start = 0;
			for (size_t I = 0; I <= Payload.size(); ++I)
			{
				if (I == Payload.size() || Payload[I] == '|') { F.push_back(Payload.substr(Start, I - Start)); Start = I + 1; }
			}
			if (F.size() != 6 || F[0] != "KRPK1") { L.Error = "Unknown key version."; return L; }
			L.Name = F[1]; L.Machine = F[2]; L.Issued = ParseYmd(F[3]); L.Expires = ParseYmd(F[4]); L.KeyId = F[5];
			if (L.Issued == 0 || L.Expires == 0) { L.Error = "The key's dates are invalid."; return L; }
			if (L.Machine != MachineCode) { L.Error = "This key belongs to another PC (machine " + L.Machine + ")."; return L; }
			if (TodayYmd > L.Expires) { L.Error = "This key expired on " + F[4].substr(0, 4) + "-" + F[4].substr(4, 2) + "-" + F[4].substr(6, 2) + "."; return L; }
			L.bValid = true;
			return L;
		}

		namespace
		{
			std::vector<std::string> Split(const std::string& S, char Sep)
			{
				std::vector<std::string> F;
				size_t Start = 0;
				for (size_t I = 0; I <= S.size(); ++I)
				{
					if (I == S.size() || S[I] == Sep) { F.push_back(S.substr(Start, I - Start)); Start = I + 1; }
				}
				return F;
			}
		}

		RevocationList ParseRevocationList(const std::string& TextIn, const std::vector<uint8_t>& ModulusBE, uint32_t E)
		{
			RevocationList R;
			std::string Text;
			for (char C : TextIn) { if (C != ' ' && C != '\r' && C != '\n' && C != '\t') { Text += C; } }
			const size_t Dot = Text.find('.');
			if (Dot == std::string::npos) { return R; }
			std::vector<uint8_t> PayloadB, Sig;
			if (!Base64UrlDecode(Text.substr(0, Dot), PayloadB) || !Base64UrlDecode(Text.substr(Dot + 1), Sig)) { return R; }
			const std::string Payload(PayloadB.begin(), PayloadB.end());
			if (!RsaVerifySha256(ModulusBE, E, Payload, Sig)) { return R; }
			const std::vector<std::string> F = Split(Payload, '|');
			if (F.size() != 5 || F[0] != "KRPKREV1") { return R; }
			R.Date = ParseYmd(F[1]);
			R.Serial = std::atoll(F[2].c_str());
			for (const std::string& S : Split(F[3], ',')) { if (!S.empty()) { R.KeyIds.push_back(S); } }
			for (const std::string& S : Split(F[4], ',')) { if (!S.empty()) { R.Machines.push_back(S); } }
			R.bValid = R.Date != 0 && R.Serial > 0;
			return R;
		}

		bool IsRevoked(const RevocationList& List, const LicenseInfo& L)
		{
			if (!List.bValid) { return false; }
			return std::find(List.KeyIds.begin(), List.KeyIds.end(), L.KeyId) != List.KeyIds.end() ||
			       std::find(List.Machines.begin(), List.Machines.end(), L.Machine) != List.Machines.end();
		}

		namespace
		{
			// Written by Tools/License/krpk_license.py init (empty = no key generated yet).
#include "LicensePublicKey.inc"
			// Written by krpk_license.py revocation-setup (empty = revocation not set up).
#include "LicenseRevocation.inc"
		}

		const char* GameRevocationUrl() { return KrpkRevocationUrl; }

		const std::vector<uint8_t>& GamePublicModulus()
		{
			static const std::vector<uint8_t> M = []()
			{
				std::vector<uint8_t> V;
				const char* Hex = KrpkLicenseModulusHex;
				for (size_t I = 0; Hex[I] && Hex[I + 1]; I += 2)
				{
					auto Nib = [](char C) { return C >= 'a' ? C - 'a' + 10 : (C >= 'A' ? C - 'A' + 10 : C - '0'); };
					V.push_back(uint8_t(Nib(Hex[I]) * 16 + Nib(Hex[I + 1])));
				}
				return V;
			}();
			return M;
		}
		uint32_t GamePublicExponent() { return KrpkLicenseExponent; }
		bool HasGamePublicKey() { return GamePublicModulus().size() >= 256; }
	}
}
