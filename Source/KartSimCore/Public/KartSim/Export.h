// Symbol export for the kart simulation library. Inside Unreal, UBT defines KARTSIMCORE_API
// (dllexport/dllimport for modular builds); in standalone builds it expands to nothing.
#pragma once

#if defined(KARTSIMCORE_API)
#define KARTSIM_API KARTSIMCORE_API
#else
#define KARTSIM_API
#endif
