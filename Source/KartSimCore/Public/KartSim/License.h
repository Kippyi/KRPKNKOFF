// Tester licensing (engine-free so the standalone tests can check it against the key tool).
//
// A license key is   base64url(payload) "." base64url(signature)
//   payload   = "KRPK1|<name>|<machine code>|<issued YYYYMMDD>|<expires YYYYMMDD>|<key id>"
//   signature = RSASSA-PKCS1-v1_5 / SHA-256 over the payload bytes, made with the developer's PRIVATE key by
//               Tools/License/krpk_license.py. The game only holds the PUBLIC key (LicensePublicKey.inc), so it can
//               check keys but never make them.
// The machine code binds a key to one PC (the game derives it from Windows' machine GUID and the system volume
// serial). Expiry is checked against max(today, last date this PC ran the game), so winding the clock back does not
// extend a key.
//
// Honest limits: this stops casual sharing (a key does not work on another PC, the tester's name is watermarked on
// screen). A skilled cracker can patch the check out of the executable; no offline scheme prevents that.
#pragma once

#include "KartSim/Export.h"

#include <cstdint>
#include <string>
#include <vector>

namespace kartsim
{
	namespace license
	{
		KARTSIM_API void Sha256(const uint8_t* Data, size_t Len, uint8_t Out[32]);
		KARTSIM_API std::string Sha256Hex(const std::string& Text);
		KARTSIM_API bool Base64UrlDecode(const std::string& In, std::vector<uint8_t>& Out);
		// RSASSA-PKCS1-v1_5 with SHA-256. Modulus big-endian, public exponent E (65537).
		KARTSIM_API bool RsaVerifySha256(const std::vector<uint8_t>& ModulusBE, uint32_t E, const std::string& Message, const std::vector<uint8_t>& Signature);
		// 16 characters of base32 (80 bits of SHA-256 over the machine identity), as XXXX-XXXX-XXXX-XXXX.
		KARTSIM_API std::string MachineCodeFrom(const std::string& MachineIdentity);

		struct LicenseInfo
		{
			bool bValid = false;
			std::string Error;       // why not valid (shown to the tester)
			std::string Name, Machine, KeyId;
			int Issued = 0, Expires = 0; // YYYYMMDD
		};

		// Checks a key against the public key, this PC's machine code and the date (YYYYMMDD).
		KARTSIM_API LicenseInfo CheckLicense(const std::string& Key, const std::vector<uint8_t>& ModulusBE, uint32_t E, const std::string& MachineCode, int TodayYmd);

		// Revocation list, published by the developer (krpk_license.py revoke / the Discord bot) at a URL compiled into the
		// game. Same signing key as the licenses:  base64url(payload) "." base64url(signature)
		//   payload = "KRPKREV1|<YYYYMMDD>|<serial>|<key id>,<key id>,...|<machine>,<machine>,..."
		// Serial grows with every publish, so the game can refuse an older list (replay).
		struct RevocationList
		{
			bool bValid = false;
			int Date = 0;
			long long Serial = 0;
			std::vector<std::string> KeyIds, Machines;
		};
		KARTSIM_API RevocationList ParseRevocationList(const std::string& Text, const std::vector<uint8_t>& ModulusBE, uint32_t E);
		KARTSIM_API bool IsRevoked(const RevocationList& List, const LicenseInfo& License);

		// The game's embedded public key (LicensePublicKey.inc, written by `krpk_license.py init`).
		KARTSIM_API const std::vector<uint8_t>& GamePublicModulus();
		KARTSIM_API uint32_t GamePublicExponent();
		KARTSIM_API bool HasGamePublicKey();
		// Where the game downloads the revocation list (empty = revocation not set up; LicenseRevocation.inc).
		KARTSIM_API const char* GameRevocationUrl();
	}
}
