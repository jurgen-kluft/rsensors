#include "rcore/c_serial.h"
#include "rcore/c_str.h"
#include "ccore/c_memory.h"

#include "lib_hmmd/c_hmmd.h"

namespace ncore
{
    namespace nsensors
    {
        bool initHMMD(u8 rxPin, u8 txPin)
        {
            nserialx::begin(nserialx::SERIAL2, nbaud::Rate115200, nconfig::MODE_8N1, rxPin, txPin);

            // // Put the sensor into 'normal' mode and keep it active
            // const byte command_normal_mode[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};
            // nserialx::write(command_normal_mode, sizeof(command_normal_mode));

            // Put the sensor into 'report' mode and keep it active
            const byte command_report_mode[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};
            nserialx::write(nserialx::SERIAL2, command_report_mode, sizeof(command_report_mode));

            return true;
        }

        // Report Mode
        //   Byte 1~4       Byte5,6   Byte 7,8    Byte9~10    Byte 11~14    Byte 15~18
        //   FD FC FB FA    08 00     12 00       00 00       04 00 00 00   04 03 02 01
        //
        // In report mode serial port data frame example:
        //
        // Frame Header    Length   Detection Result   Target Distance   The energy values for each distance gate    Frame Tail
        // 4 bytes,        2 bytes, 1 byte,            2 bytes,          32 bytes,                                   4 bytes
        //
        //   Frame Header (F4 F3 F2 F1)
        //   Detection Result (00 absent, 01 present)
        //   Length, total number of bytes for detection result, target distance, and energy values for each distance gate
        //   Target Distance, indicating the distance of the target phase from the radar in the scene
        //   Energy values, 16 (total number of distance gates) * 2 bytes, size of energy value for each distance gate from 0 to 15
        //   Frame Tail (F8 F7 F6 F5)
        // Total data frame length = 4 + 2 + 1 + 2 + 32 + 4 = 45 bytes

// Rx buffer size must be a power of two for the circular buffer logic to work correctly
#define hmmdRxBufferSize     256
#define hmmdRxBufferSizeMask 255
#define hmmdFrameLength      45

        static inline bool matchesFrameHeader(const byte* buffer, s32 pos)
        {
            u32 header = buffer[pos];
            pos        = (pos + 1) & hmmdRxBufferSizeMask;
            header     = (header << 8) | buffer[pos];
            pos        = (pos + 1) & hmmdRxBufferSizeMask;
            header     = (header << 8) | buffer[pos];
            pos        = (pos + 1) & hmmdRxBufferSizeMask;
            header     = (header << 8) | buffer[pos];
            return (header == 0xF4F3F2F1);
        }

        static inline bool matchesFrameTail(const byte* buffer, s32 tailPos, s32 frameSize)
        {
            u32 tail = buffer[tailPos];
            tailPos  = (tailPos + 1) & hmmdRxBufferSizeMask;
            tail     = (tail << 8) | buffer[tailPos];
            tailPos  = (tailPos + 1) & hmmdRxBufferSizeMask;
            tail     = (tail << 8) | buffer[tailPos];
            tailPos  = (tailPos + 1) & hmmdRxBufferSizeMask;
            tail     = (tail << 8) | buffer[tailPos];
            return (tail == 0xF8F7F6F5);
        }

        // Circular 'moving window' buffer for receiving data from the HMMD sensor
        byte hmmdRxBuffer[hmmdRxBufferSize];
        s32  hmmdRxBufferReadPos  = 0;
        s32  hmmdRxBufferWritePos = 0;

        // Read data from the HMMD sensor in 'report' mode
        bool readHMMD2(s8* outDetection, u16* outDistanceInCm)
        {
            // Drain data from Serial2
            s32 available = nserialx::available(nserialx::SERIAL2);
            while (available > 0)
            {
                // Calculate the current length of read data in the buffer, taking into account wrap-around of the circular buffer
                s32 currentFrameLength = (hmmdRxBufferWritePos + hmmdRxBufferSize - hmmdRxBufferReadPos) & hmmdRxBufferSizeMask;

                // Read available bytes into the buffer
                while (available > 0 && currentFrameLength < hmmdRxBufferSize)
                {
                    // Determine how many bytes we can read without overflowing the circular buffer
                    const s32 numBytesCanRead = hmmdRxBufferWritePos >= hmmdRxBufferReadPos ? hmmdRxBufferSize - hmmdRxBufferWritePos : hmmdRxBufferReadPos - hmmdRxBufferWritePos;
                    const s32 numBytesRead    = static_cast<s32>(nserialx::read_bytes(nserialx::SERIAL2, &hmmdRxBuffer[hmmdRxBufferWritePos], numBytesCanRead <= available ? numBytesCanRead : available));
                    if (numBytesRead == 0)
                    {
                        available = -1;  // Error condition, exit the loop
                        break;
                    }
                    hmmdRxBufferWritePos = (hmmdRxBufferWritePos + numBytesRead) & hmmdRxBufferSizeMask;
                    currentFrameLength   = currentFrameLength + numBytesRead;
                    available            = available - numBytesRead;
                }

                // When the above read loop exits and available == -1, it indicates an error condition.
                // In that case, we skip the processing of the buffer and will loop back to the top to try reading again.
                if (available >= 0)
                {
                    // Advance the read position until we find a valid frame header
                    bool foundHeader = false;
                    while (currentFrameLength >= hmmdFrameLength)
                    {
                        if (hmmdRxBuffer[hmmdRxBufferReadPos] == 0xF4 && matchesFrameHeader(hmmdRxBuffer, hmmdRxBufferReadPos))
                        {
                            foundHeader = true;  // Found a valid frame header
                            break;
                        }
                        hmmdRxBufferReadPos = (hmmdRxBufferReadPos + 1) & hmmdRxBufferSizeMask;  // Move read position forward by one byte
                        currentFrameLength--;                                                    // Decrease the current frame length
                    }

                    if (foundHeader && currentFrameLength >= hmmdFrameLength)
                    {
                        const s32 tailPos = (hmmdRxBufferReadPos + hmmdFrameLength - 4) & hmmdRxBufferSizeMask;
                        if (hmmdRxBuffer[tailPos] == 0xF8 && matchesFrameTail(hmmdRxBuffer, tailPos, hmmdFrameLength))
                        {
                            // Extract the detection result (byte 6)
                            const s32 detectionPos = (hmmdRxBufferReadPos + 6) & hmmdRxBufferSizeMask;
                            *outDetection          = static_cast<s8>(hmmdRxBuffer[detectionPos]);

                            // Extract the target distance (bytes 7 and 8)
                            const s32 distancePosL = (hmmdRxBufferReadPos + 7) & hmmdRxBufferSizeMask;
                            const s32 distancePosH = (hmmdRxBufferReadPos + 8) & hmmdRxBufferSizeMask;
                            *outDistanceInCm       = static_cast<u16>(hmmdRxBuffer[distancePosL]) | (static_cast<u16>(hmmdRxBuffer[distancePosH]) << 8);

                            // Move read position to the next frame
                            hmmdRxBufferReadPos += hmmdFrameLength;

                            return true;
                        }

                        // We have a valid frame header but invalid frame tail?
                        // Skip this full frame and continue searching for the next frame header
                        hmmdRxBufferReadPos = (hmmdRxBufferReadPos + hmmdFrameLength) & hmmdRxBufferSizeMask;
                    }
                }

                available = nserialx::available(nserialx::SERIAL2);
            }
            return false;
        }

        // Read data from the HMMD sensor in 'normal' mode
        bool readHMMD(s8* outDetection, u16* outDistanceInCm)
        {
            char line[128];
            g_memset(line, 0, sizeof(line));

            // Drain data from Serial2
            s32 available = nserialx::available(nserialx::SERIAL2);
            while (available > 0)
            {
                // Read a line of text until a newline character (\n) is received
                s32 lineLength = nserialx::read_until(nserialx::SERIAL2, '\n', line, 128 - 1);
                if (lineLength == 0)
                {
                    break;  // No more data available
                }

                // Clean up the line: remove potential carriage return (\r) and leading/trailing whitespace
                char* begin = line;
                char* end   = line + lineLength;
                while (end > begin && (*(end - 1) == '\r' || *(end - 1) == '\n' || *(end - 1) == ' '))
                {
                    end = end - 1;
                }
                while (begin < end && (*begin == ' ' || *begin == '\r' || *begin == '\n'))
                {
                    begin = begin + 1;
                }

                // Check if the line contains the "Range" information
                lineLength = static_cast<s32>(end - begin);
                str_t str  = str_const_n(begin, lineLength);
                if (lineLength > 6 && str_cmp_n(str, "Range ", 6) == 0)
                {
                    str.m_str += 6;
                    s32 distance = 0;
                    if (from_str(str, &distance, 10))
                    {
                        *outDistanceInCm = (u16)distance;
                        return true;
                    }
                }

                available = nserialx::available(nserialx::SERIAL2);
            }

            return false;
        }
    }  // namespace nsensors
}  // namespace ncore
