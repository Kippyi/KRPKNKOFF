# Tester licensing and build protection (2026-11-20)

User: "I want to start getting testers and I don't want them passing it around ... encrypt the code so people can't
steal my game". Chosen: own license keys now, Steam Playtest later.

## Day-to-day: adding a tester
1. Send them the tester zip (`Build\KRPKNKOFF_Tester_<date>.zip`, made by `Tools\PackageTesterBuild.bat`).
2. They start it and send you the MACHINE CODE from the activation screen (Copy button).
3. You run (in E:\KRPKNKOFF):
   `python Tools\License\krpk_license.py issue --name "Their Name" --machine ABCD-EFGH-JKLM-NPQR --days 30`
   and send them the key it prints (one long line). They press Paste, then Activate.
4. `python Tools\License\krpk_license.py list` shows every key you issued (name, PC, dates, key id).
   Renew = issue a new key for the same machine code. A tester who reinstalls Windows gets a new machine code.

## The signing key (IMPORTANT)
* Private key: `C:\Users\<you>\KRPKNKOFF_Signing\license_private_key.pem` (+ `issued_keys.csv`). Created 2026-11-20
  by `krpk_license.py init` WITHOUT a passphrase. Anyone who has this file can make keys for your game.
  Back it up offline (USB stick / password manager). Never put it in the game folder, a zip, a repo or a chat.
* Optional, before issuing real tester keys: re-create it with a passphrase:
  `python Tools\License\krpk_license.py init --force --passphrase`, then rebuild + repackage (the game embeds the
  public half, `Source/KartSimCore/Private/KartSim/LicensePublicKey.inc`). This invalidates all keys made so far
  (including your own "Developer" key: issue yourself a new one).

## What the protection does
| Threat | Protection | Limit |
|---|---|---|
| Tester passes the zip to a friend | Key bound to one PC (Windows machine GUID + system volume serial) | Friend can ask you for a key; you decide |
| Leaked build / stream | Name + machine code watermarked at the bottom of the screen; issued_keys.csv maps it to the tester | Watermark can be cropped from videos |
| Using it forever | Keys expire; date = max(today, last date seen on that PC) so winding the clock back does not extend it | Deleting the state file in %LOCALAPPDATA% resets "last seen" |
| Reading / editing physics data, ripping models | Kart / track JSON moved into the pak; pak AES-256 encrypted (Config/DefaultCrypto.ini) | The AES key is inside the exe; known UE tools can dig it out |
| Reading the code | Shipping build: native code only, no .pdb symbols, no console, developer -Kart flags stripped | A disassembler still works; no client-side protection stops a determined cracker |
| Forging keys | RSA-2048 signatures (PKCS#1 v1.5, SHA-256); the game only has the public key | Patching the check out of the exe bypasses it |

Honest summary: this stops casual sharing and makes leaks traceable. It does not stop a skilled cracker, and no
offline scheme does. The strong protections are legal (a tester agreement / NDA) and later Steam (Valve's DRM, revocable
keys, server-side ownership). Your source code (E:\KRPKNKOFF) is never shipped; keep it backed up and private.

## Build types
* `Tools\PackageGame.bat` -> Build\Windows, Development: for you and my automated tests. License enforced only with
  `-KartLicenseCheck`. Keep this zip private (console + test flags + .pdb symbols).
* `Tools\PackageTesterBuild.bat` -> Build\Tester\Windows + `KRPKNKOFF_Tester_<date>.zip`, Shipping: license always
  enforced, no .pdb, no console, -Kart test flags removed except -KartOldEngineSound / -KartNoWheel / -KartGraphics=.
  Adds `steam_appid.txt` (480, Valve's test app) so Steam multiplayer starts in Shipping until KRPKNKOFF has its own
  app id. Tester feedback log: `%LOCALAPPDATA%\KRPKNKOFF\Saved\Logs\KartTester.log` (license state, Steam status,
  damage / contact events; UE's own log is compiled out in Shipping).

## Code
* `KartSim/License.h/.cpp` (core, engine-free): SHA-256, RSA PKCS#1 v1.5 verify (own bignum), key parsing, machine
  code format. Tests `License_*`: FIPS SHA-256 vectors, keys made by the Python tool (`testvectors`) verified / forged /
  tampered / other PC / expired / junk.
* `KartLicense.h/.cpp` (game): machine identity, key + last-seen state in `%LOCALAPPDATA%\KRPKNKOFF\License`,
  activation screen (Slate), watermark, `KartTesterLog`. Gates: activation screen instead of the menu (KartHUD), no
  physics steps while blocked (KartRaceWorld, KartVehiclePawn).
* `KRPKNKOFF.cpp`: module startup installs a UE file reader for the core (pak access) and strips test flags in Shipping.

## Discord license bot (2026-11-20)
User: "can I get the licensing creator on a Discord bot?" `Tools/License/krpk_discord_bot.py` runs on the developer's
PC (the private key never leaves it; testers can only get keys while it runs). Own Python environment
`Tools/License/.venv` (discord.py 2.6.0 + cryptography): the global Python has nextcord AND discord.py installed, which
overwrite each other's `discord` package (import of discord.app_commands fails there).
* Tester: `/activate code:<machine code>` (needs the Tester role) -> key only they can see; Discord display name =
  watermark name; same PC again = same key, renewed within 7 days of expiry; max 2 PCs per member; a PC registered to
  another member is refused. `/mykeys` re-shows their keys.
* Admin (server owner, admin role, or admin_user_ids): `/license issue | list | block | unblock | check`.
* Optional private log channel: "Key for @member (PC ...)" (never the key).
* Ledger shared with the command-line tool (`note = discord:<user id>`); bot state in discord_state.json; token in
  discord_bot.json - all in the KRPKNKOFF_Signing folder.
* Verified offline: `--selftest` (12 rules), dry start registering /activate, /mykeys, /license (5 subcommands).
  NOT verified against Discord itself (needs the user's bot token).

### One-time setup
1. https://discord.com/developers/applications > New Application (e.g. "KRPKNKOFF Licenses") > Bot > Reset Token >
   copy it (it is a password). No privileged intents needed.
2. OAuth2 > URL Generator: scopes `bot` + `applications.commands`, permission `Send Messages` (only for the log
   channel). Open the URL, add the bot to your server.
3. In your server: create a `Tester` role (and give it to testers), optionally a `Dev` role for helpers.
   Discord Settings > Advanced > Developer Mode on; right-click the server icon > Copy Server ID.
4. Double-click `Tools\License\Setup_LicenseBot.bat`: paste the token (hidden) and the server id.
5. Double-click `Tools\License\Start_LicenseBot.bat` and leave the window open.

## Revoking access (2026-11-20)
User: "can I have a way to revoke someone's access?" Keys are checked offline, so revocation needs the game to look
something up: a REVOCATION LIST of key ids and machine codes, signed with the same private key (it cannot be forged or
edited; any public host works). Default host: a secret GitHub Gist that the tool / bot updates by itself.
* Game (`FKartLicense`): applies the cached list at start, downloads the current list in the background
  (`HTTP` module, 15 s timeout); a revoked key / PC -> activation screen ("This license was revoked by the developer"),
  also mid-session when the answer arrives. Lists with a lower serial than the newest seen are ignored (replay).
  Offline: allowed for 7 days after the last successful check (`GraceDays`); a key never checked online needs one
  check. State (`license.state` v2): last date seen, last check, newest serial, with a checksum.
* Tool: `revocation-setup --github` (creates the Gist, asks for a GitHub token with the "gist" scope, writes the URL into
  `LicenseRevocation.inc`) or `--url URL` (host krpk_revoked.txt yourself); `revoke --key-id/--machine/--name`,
  `unrevoke`, `revoked`. Serial = Unix time of the publish.
* Bot: `/license revoke member | key_id | machine | name` (a member: all their keys + PCs, and blocked), `/license unrevoke`.
* Verified: C++ against tool-made lists (by id / by machine / others / forged); bot selftest 18 rules; in game against a
  local server with lists signed by the real key: clean list valid, revoked -> activation screen + "withdrawn during
  the session", older list replay ignored, newer list lifts it, server down within grace valid. Test lists used serials
  1000-1004 (real lists use Unix time, so they are always newer).
* NOT verified: the GitHub Gist upload (needs the user's GitHub token). Until `revocation-setup` is run and the tester
  build repackaged, the URL is empty and nothing is checked.
