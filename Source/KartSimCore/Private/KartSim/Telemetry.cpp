#include "KartSim/Telemetry.h"

namespace kartsim
{
	namespace
	{
		FILE* OpenForWrite(const std::string& Path)
		{
			FILE* F = nullptr;
#if defined(_MSC_VER)
			if (fopen_s(&F, Path.c_str(), "wb") != 0) { F = nullptr; }
#else
			F = std::fopen(Path.c_str(), "wb");
#endif
			return F;
		}
	}

	TelemetryBuffer::TelemetryBuffer(size_t InCapacity)
		: Frames(InCapacity > 0 ? InCapacity : 1)
	{
	}

	void TelemetryBuffer::Push(const TelemetryFrame& Frame)
	{
		Frames[Head] = Frame;
		Head = (Head + 1) % Frames.size();
		if (Count < Frames.size()) { ++Count; }
	}

	void TelemetryBuffer::Clear()
	{
		Head = 0;
		Count = 0;
	}

	const TelemetryFrame& TelemetryBuffer::At(size_t Index) const
	{
		const size_t Start = (Head + Frames.size() - Count) % Frames.size();
		return Frames[(Start + Index) % Frames.size()];
	}

	bool TelemetryBuffer::WriteCsv(const std::string& Path) const
	{
		FILE* F = OpenForWrite(Path);
		if (!F)
		{
			return false;
		}
		CsvTelemetryWriter::WriteHeader(F);
		for (size_t I = 0; I < Count; ++I)
		{
			CsvTelemetryWriter::WriteRow(F, At(I));
		}
		std::fclose(F);
		return true;
	}

	bool CsvTelemetryWriter::Open(const std::string& Path)
	{
		Close();
		File = OpenForWrite(Path);
		if (File)
		{
			WriteHeader(File);
		}
		return File != nullptr;
	}

	void CsvTelemetryWriter::Write(const TelemetryFrame& Frame)
	{
		if (File)
		{
			WriteRow(File, Frame);
		}
	}

	void CsvTelemetryWriter::Close()
	{
		if (File)
		{
			std::fclose(File);
			File = nullptr;
		}
	}

	void CsvTelemetryWriter::WriteHeader(FILE* F)
	{
		bool bFirst = true;
#define KARTSIM_WRITE_NAME(Name) std::fprintf(F, bFirst ? "%s" : ",%s", #Name); bFirst = false;
		KARTSIM_TELEMETRY_FIELDS(KARTSIM_WRITE_NAME)
#undef KARTSIM_WRITE_NAME
		std::fprintf(F, "\n");
	}

	void CsvTelemetryWriter::WriteRow(FILE* F, const TelemetryFrame& Frame)
	{
		bool bFirst = true;
#define KARTSIM_WRITE_VALUE(Name) std::fprintf(F, bFirst ? "%.6g" : ",%.6g", Frame.Name); bFirst = false;
		KARTSIM_TELEMETRY_FIELDS(KARTSIM_WRITE_VALUE)
#undef KARTSIM_WRITE_VALUE
		std::fprintf(F, "\n");
	}
}
