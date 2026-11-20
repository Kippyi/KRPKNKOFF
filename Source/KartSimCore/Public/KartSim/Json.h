// Minimal JSON reader with // and /* */ comment support (JSONC), used for kart configs.
// No exceptions (Unreal builds with exceptions disabled); errors are reported via strings.
#pragma once

#include "KartSim/Export.h"

#include <string>
#include <utility>
#include <vector>

namespace kartsim
{
	class KARTSIM_API JsonValue
	{
	public:
		enum class EType { Null, Bool, Number, String, Array, Object };

		EType Type = EType::Null;
		bool BoolValue = false;
		double NumberValue = 0.0;
		std::string StringValue;
		std::vector<JsonValue> ArrayValue;
		std::vector<std::pair<std::string, JsonValue>> ObjectValue;

		bool IsNull() const { return Type == EType::Null; }
		bool IsObject() const { return Type == EType::Object; }
		bool IsArray() const { return Type == EType::Array; }
		bool IsNumber() const { return Type == EType::Number; }

		// Returns nullptr if missing or not an object.
		const JsonValue* Find(const char* Key) const;

		// Dotted path lookup ("engine.inertia").
		const JsonValue* FindPath(const char* Path) const;
	};

	struct JsonParseResult
	{
		bool bOk = false;
		std::string Error;
		JsonValue Root;
	};

	KARTSIM_API JsonParseResult ParseJson(const std::string& Text);

	// Reads an entire file into a string. Returns false on failure.
	KARTSIM_API bool ReadTextFile(const std::string& Path, std::string& OutText);
	// The game installs a reader that goes through UE's file system (pak files); the tests use plain fopen.
	using TextFileReader = bool (*)(const std::string& Path, std::string& OutText);
	KARTSIM_API void SetTextFileReader(TextFileReader Reader);
}
