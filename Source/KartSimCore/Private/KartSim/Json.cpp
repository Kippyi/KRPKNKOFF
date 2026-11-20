#include "KartSim/Json.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace kartsim
{
	const JsonValue* JsonValue::Find(const char* Key) const
	{
		if (Type != EType::Object)
		{
			return nullptr;
		}
		for (const auto& Pair : ObjectValue)
		{
			if (Pair.first == Key)
			{
				return &Pair.second;
			}
		}
		return nullptr;
	}

	const JsonValue* JsonValue::FindPath(const char* Path) const
	{
		const JsonValue* Node = this;
		std::string Segment;
		for (const char* C = Path;; ++C)
		{
			if (*C == '.' || *C == '\0')
			{
				Node = Node->Find(Segment.c_str());
				if (!Node || *C == '\0')
				{
					return Node;
				}
				Segment.clear();
			}
			else
			{
				Segment.push_back(*C);
			}
		}
	}

	namespace
	{
		struct JsonParser
		{
			const std::string& Src;
			size_t Pos = 0;
			int Line = 1;
			std::string Error;

			explicit JsonParser(const std::string& InSrc) : Src(InSrc) {}

			bool Fail(const char* Msg)
			{
				if (Error.empty())
				{
					char Buf[256];
					std::snprintf(Buf, sizeof(Buf), "JSON error at line %d: %s", Line, Msg);
					Error = Buf;
				}
				return false;
			}

			void SkipWs()
			{
				while (Pos < Src.size())
				{
					const char C = Src[Pos];
					if (C == '\n') { ++Line; ++Pos; }
					else if (C == ' ' || C == '\t' || C == '\r') { ++Pos; }
					else if (C == '/' && Pos + 1 < Src.size() && Src[Pos + 1] == '/')
					{
						while (Pos < Src.size() && Src[Pos] != '\n') { ++Pos; }
					}
					else if (C == '/' && Pos + 1 < Src.size() && Src[Pos + 1] == '*')
					{
						Pos += 2;
						while (Pos + 1 < Src.size() && !(Src[Pos] == '*' && Src[Pos + 1] == '/'))
						{
							if (Src[Pos] == '\n') { ++Line; }
							++Pos;
						}
						Pos += 2;
					}
					else { break; }
				}
			}

			bool ParseString(std::string& Out)
			{
				if (Src[Pos] != '"') { return Fail("expected string"); }
				++Pos;
				while (Pos < Src.size() && Src[Pos] != '"')
				{
					char C = Src[Pos++];
					if (C == '\\' && Pos < Src.size())
					{
						const char E = Src[Pos++];
						switch (E)
						{
						case 'n': C = '\n'; break;
						case 't': C = '\t'; break;
						case 'r': C = '\r'; break;
						case 'b': C = '\b'; break;
						case 'f': C = '\f'; break;
						case 'u': Pos += 4; C = '?'; break; // configs are ASCII; unicode escapes not needed
						default: C = E; break;
						}
					}
					Out.push_back(C);
				}
				if (Pos >= Src.size()) { return Fail("unterminated string"); }
				++Pos;
				return true;
			}

			bool ParseValue(JsonValue& Out, int Depth)
			{
				if (Depth > 64) { return Fail("nesting too deep"); }
				SkipWs();
				if (Pos >= Src.size()) { return Fail("unexpected end of input"); }
				const char C = Src[Pos];
				if (C == '{')
				{
					Out.Type = JsonValue::EType::Object;
					++Pos;
					SkipWs();
					if (Pos < Src.size() && Src[Pos] == '}') { ++Pos; return true; }
					for (;;)
					{
						SkipWs();
						std::string Key;
						if (!ParseString(Key)) { return false; }
						SkipWs();
						if (Pos >= Src.size() || Src[Pos] != ':') { return Fail("expected ':'"); }
						++Pos;
						JsonValue Child;
						if (!ParseValue(Child, Depth + 1)) { return false; }
						Out.ObjectValue.emplace_back(std::move(Key), std::move(Child));
						SkipWs();
						if (Pos < Src.size() && Src[Pos] == ',')
						{
							++Pos;
							SkipWs();
							if (Pos < Src.size() && Src[Pos] == '}') { ++Pos; return true; } // trailing comma
							continue;
						}
						if (Pos < Src.size() && Src[Pos] == '}') { ++Pos; return true; }
						return Fail("expected ',' or '}'");
					}
				}
				if (C == '[')
				{
					Out.Type = JsonValue::EType::Array;
					++Pos;
					SkipWs();
					if (Pos < Src.size() && Src[Pos] == ']') { ++Pos; return true; }
					for (;;)
					{
						JsonValue Child;
						if (!ParseValue(Child, Depth + 1)) { return false; }
						Out.ArrayValue.push_back(std::move(Child));
						SkipWs();
						if (Pos < Src.size() && Src[Pos] == ',')
						{
							++Pos;
							SkipWs();
							if (Pos < Src.size() && Src[Pos] == ']') { ++Pos; return true; }
							continue;
						}
						if (Pos < Src.size() && Src[Pos] == ']') { ++Pos; return true; }
						return Fail("expected ',' or ']'");
					}
				}
				if (C == '"')
				{
					Out.Type = JsonValue::EType::String;
					return ParseString(Out.StringValue);
				}
				if (Src.compare(Pos, 4, "true") == 0) { Out.Type = JsonValue::EType::Bool; Out.BoolValue = true; Pos += 4; return true; }
				if (Src.compare(Pos, 5, "false") == 0) { Out.Type = JsonValue::EType::Bool; Out.BoolValue = false; Pos += 5; return true; }
				if (Src.compare(Pos, 4, "null") == 0) { Out.Type = JsonValue::EType::Null; Pos += 4; return true; }
				if (C == '-' || (C >= '0' && C <= '9'))
				{
					const char* Begin = Src.c_str() + Pos;
					char* End = nullptr;
					Out.NumberValue = std::strtod(Begin, &End);
					if (End == Begin) { return Fail("bad number"); }
					Out.Type = JsonValue::EType::Number;
					Pos += static_cast<size_t>(End - Begin);
					return true;
				}
				return Fail("unexpected character");
			}
		};
	}

	JsonParseResult ParseJson(const std::string& Text)
	{
		JsonParseResult Result;
		JsonParser Parser(Text);
		if (Text.size() >= 3 && static_cast<unsigned char>(Text[0]) == 0xEF &&
		    static_cast<unsigned char>(Text[1]) == 0xBB && static_cast<unsigned char>(Text[2]) == 0xBF)
		{
			Parser.Pos = 3; // UTF-8 byte order mark
		}
		Result.bOk = Parser.ParseValue(Result.Root, 0);
		if (Result.bOk)
		{
			Parser.SkipWs();
			if (Parser.Pos != Text.size())
			{
				Result.bOk = Parser.Fail("trailing characters after root value");
			}
		}
		Result.Error = Parser.Error;
		return Result;
	}

	namespace { TextFileReader GReader = nullptr; }
	void SetTextFileReader(TextFileReader Reader) { GReader = Reader; }

	bool ReadTextFile(const std::string& Path, std::string& OutText)
	{
		if (GReader) { return GReader(Path, OutText); }
		FILE* F = nullptr;
#if defined(_MSC_VER)
		if (fopen_s(&F, Path.c_str(), "rb") != 0) { F = nullptr; }
#else
		F = std::fopen(Path.c_str(), "rb");
#endif
		if (!F)
		{
			return false;
		}
		std::fseek(F, 0, SEEK_END);
		const long Size = std::ftell(F);
		std::fseek(F, 0, SEEK_SET);
		OutText.resize(Size > 0 ? static_cast<size_t>(Size) : 0);
		const size_t Read = Size > 0 ? std::fread(&OutText[0], 1, static_cast<size_t>(Size), F) : 0;
		std::fclose(F);
		return Read == OutText.size();
	}
}
