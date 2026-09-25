#include "WavStreamDecoder.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <ios>
#include <istream>

namespace Engine
{
    namespace
    {
        constexpr std::uint16_t WAV_PCM = 1;

        constexpr std::uint16_t WAV_IEEE_FLOAT = 3;

        bool FourCCEquals(const char* value, const char* expected)
        {
            return std::memcmp(value, expected, 4) == 0;
        }
    }

    WavStreamDecoder::~WavStreamDecoder()
    {
        Close();
    }

    std::uint16_t WavStreamDecoder::ReadU16(std::istream& stream)
    {
        std::array<std::uint8_t, 2> bytes{};

        stream.read(reinterpret_cast<char*>(bytes.data()), 2);

        return static_cast<std::uint16_t>(bytes[0]) | (static_cast<std::uint16_t>(bytes[1]) << 8);
    }

    std::uint32_t WavStreamDecoder::ReadU32(std::istream& stream)
    {
        std::array<std::uint8_t, 4> bytes{};

        stream.read(reinterpret_cast<char*>(bytes.data()), 4);

        return
            static_cast<std::uint32_t>(bytes[0]) | (static_cast<std::uint32_t>(bytes[1]) << 8) |
            (static_cast<std::uint32_t>(bytes[2]) << 16) | (static_cast<std::uint32_t>(bytes[3]) << 24);
    }

    bool WavStreamDecoder::Open(const std::string &filePath, const AudioFormat &targetFormat)
    {
        Close();

        m_File.open(filePath, std::ios::binary);

        if (!m_File.is_open())
        {
            return false;
        }

        if (!ParseHeader())
        {
            Close();

            return false;
        }

        if (m_Format.SampleRate != targetFormat.SampleRate || m_Format.Channels != targetFormat.Channels)
        {
            Close();

            return false;
        }

        m_Format = targetFormat;

        m_File.clear();

        m_File.seekg(static_cast<std::streamoff>(m_DataOffset), std::ios::beg);

        if (!m_File.good())
        {
            Close();

            return false;
        }

        m_CurrentFrame = 0;

        m_Open = true;

        return true;
    }

    bool WavStreamDecoder::ParseHeader()
    {
        char riff[4]{};

        m_File.read(riff, 4);

        if (!m_File || !FourCCEquals(riff, "RIFF"))
        {
            return false;
        }

        // RIFF size.
        ReadU32(m_File);

        char wave[4]{};

        m_File.read(wave, 4);

        if (!m_File || !FourCCEquals(wave, "WAVE"))
        {
            return false;
        }

        bool foundFormat = false;

        bool foundData = false;

        while (m_File && !foundData)
        {
            char chunkID[4]{};

            m_File.read(chunkID, 4);

            if (!m_File)
            {
                break;
            }

            const std::uint32_t chunkSize = ReadU32(m_File);

            if (FourCCEquals(chunkID, "fmt "))
            {
                if (chunkSize < 16)
                {
                    return false;
                }

                m_WavFormatTag = ReadU16(m_File);

                const std::uint16_t channels = ReadU16(m_File);

                const std::uint32_t sampleRate = ReadU32(m_File);

                // Bytes rate.
                ReadU32(m_File);

                m_BlockAlign = ReadU16(m_File);

                m_BitsPerSample = ReadU16(m_File);

                if (chunkSize > 16)
                {
                    m_File.seekg(static_cast<std::streamoff>(chunkSize - 16), std::ios::cur);
                }

                if (channels == 0 || sampleRate == 0 || m_BlockAlign == 0)
                {
                    return false;
                }

                const bool supportedPCM16 = m_WavFormatTag == WAV_PCM && m_BitsPerSample == 16;

                const bool supporedFloat32 = m_WavFormatTag == WAV_IEEE_FLOAT && m_BitsPerSample == 32;

                if (!supportedPCM16 && !supporedFloat32)
                {
                    return false;
                }

                m_Format.SampleRate = static_cast<int>(sampleRate);

                m_Format.Channels = static_cast<int>(channels);

                foundFormat = true;
            }
            else if (FourCCEquals(chunkID, "data"))
            {
                if (!foundFormat)
                {
                    return false;
                }

                m_DataOffset = static_cast<std::uint64_t>(m_File.tellg());

                m_DataSize = chunkSize;

                foundData = true;

                break;
            }
            else
            {
                m_File.seekg(static_cast<std::streamoff>(chunkSize), std::ios::cur);
            }

            // RIFF chunks are word aligned.
            if (chunkSize & 1u)
            {
                m_File.seekg(1, std::ios::cur);
            }
        }

        if (!foundFormat || !foundData)
        {
            return false;
        }

        m_TotalFrames = m_DataSize / m_BlockAlign;

        return m_TotalFrames > 0;
    }

    void WavStreamDecoder::Close()
    {
        m_Open = false;

        if (m_File.is_open())
        {
            m_File.close();
        }

        m_Format = AudioFormat{};

        m_WavFormatTag = 0;

        m_BitsPerSample = 0;

        m_BlockAlign = 0;

        m_DataOffset = 0;

        m_DataSize = 0;

        m_TotalFrames = 0;

        m_CurrentFrame = 0;

        m_ReadScratch.clear();
    }

    bool WavStreamDecoder::IsOpen() const
    {
        return m_Open;
    }

    const AudioFormat& WavStreamDecoder::GetFormat() const
    {
        return m_Format;
    }

    std::size_t WavStreamDecoder::GetTotalFrameCount() const
    {
        return m_TotalFrames;
    }

    std::size_t WavStreamDecoder::GetCurrentFrame() const
    {
        return m_CurrentFrame;
    }

    std::size_t WavStreamDecoder::DecodeFrames(float* output, std::size_t maxFrames)
    {
        if (!m_Open || !output || maxFrames == 0 || m_CurrentFrame >= m_TotalFrames)
        {
            return 0;
        }

        const std::uint64_t remainingFrames = m_TotalFrames - m_CurrentFrame;

        const std::size_t framesToRead = static_cast<std::size_t>(std::min<std::uint64_t>(remainingFrames, maxFrames));

        const std::size_t bytesToRead = framesToRead * m_BlockAlign;

        if (m_ReadScratch.size() < bytesToRead)
        {
            // Worker-thread allocation only.
            m_ReadScratch.resize(bytesToRead);
        }

        m_File.read(reinterpret_cast<char*>(m_ReadScratch.data()), static_cast<std::streamsize>(bytesToRead));

        const std::streamsize bytesRead = m_File.gcount();

        const std::size_t actualFrames = static_cast<std::size_t>(bytesRead) / m_BlockAlign;

        if (actualFrames == 0)
        {
            return 0;
        }

        const std::size_t channels = static_cast<std::size_t>(m_Format.Channels);

        if (m_WavFormatTag == WAV_PCM && m_BitsPerSample == 16)
        {
            for (std::size_t frame = 0; frame < actualFrames; ++frame)
            {
                for (std::size_t channel = 0; channel < channels; ++channel)
                {
                    const std::size_t sampleIndex = frame * channels + channel;

                    const std::size_t byteIndex = sampleIndex * 2;

                    const std::uint16_t raw = static_cast<std::uint16_t>(m_ReadScratch[byteIndex]) | (static_cast<std::uint16_t>(m_ReadScratch[byteIndex + 1]) << 8);

                    const std::int16_t sample = static_cast<std::int16_t>(raw);

                    output[sampleIndex] = static_cast<float>(sample) / 32768.0f;
                }
            }
        }
        else if (m_WavFormatTag == WAV_IEEE_FLOAT && m_BitsPerSample == 32)
        {
            for (std::size_t frame = 0; frame < actualFrames; ++frame)
            {
                for (std::size_t channel = 0; channel < channels; ++channel)
                {
                    const std::size_t sampleIndex = frame * channels + channel;

                    const std::size_t byteIndex = sampleIndex * 4;

                    const std::uint32_t raw = static_cast<std::uint32_t>(m_ReadScratch[byteIndex]) | (static_cast<std::uint32_t>(m_ReadScratch[byteIndex + 1]) << 8) | (static_cast<std::uint32_t>(m_ReadScratch[byteIndex + 2]) << 16) | (static_cast<std::uint32_t>(m_ReadScratch[byteIndex + 3]) << 24);

                    float sample = 0.0f;

                    std::memcpy(&sample, &raw, sizeof(float));

                    output[sampleIndex] = sample;
                }
            }
        }

        m_CurrentFrame += actualFrames;

        return actualFrames;
    }

    bool WavStreamDecoder::SeekFrame(std::uint64_t frame)
    {
        if (!m_Open)
        {
            return false;
        }

        frame = std::min(frame, m_TotalFrames);

        const std::uint64_t byteOffset = m_DataOffset + frame * m_BlockAlign;

        m_File.clear();

        m_File.seekg(static_cast<std::streamoff>(byteOffset), std::ios::beg);

        if (!m_File.good())
        {
            return false;
        }

        m_CurrentFrame = frame;

        return true;
    }
}
