#pragma once

#include "AudioStreamDecoder.h"

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace Engine
{
    class WavStreamDecoder final : public AudioStreamDecoder
    {
    public:

        WavStreamDecoder() = default;


        ~WavStreamDecoder() override;


        bool Open(const std::string& filePath, const AudioFormat& targetFormat) override;


        void Close() override;


        bool IsOpen() const override;


        const AudioFormat& GetFormat() const override;


        std::uint64_t GetTotalFrameCount() const override;


        std::uint64_t GetCurrentFrame() const override;


        std::size_t DecodeFrames(float* output, std::size_t maxFrames) override;


        bool SeekFrame(std::uint64_t frame) override;


    private:

        bool ParseHeader();


        static std::uint16_t ReadU16(std::istream& stream);


        static std::uint32_t ReadU32(std::istream& stream);


    private:

        std::ifstream m_File;


        AudioFormat m_Format;


        std::uint16_t m_WavFormatTag = 0;

        std::uint16_t m_BitsPerSample = 0;

        std::uint16_t m_BlockAlign = 0;

        std::uint64_t m_DataOffset = 0;

        std::uint64_t m_DataSize = 0;

        std::uint64_t m_TotalFrames = 0;

        std::uint64_t m_CurrentFrame = 0;

        bool m_Open = false;

        std::vector<std::uint8_t> m_ReadScratch;
    };
}
