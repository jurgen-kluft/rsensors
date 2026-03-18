#ifndef __ARDUINO_SENSORS_FRAME_READER_H__
#define __ARDUINO_SENSORS_FRAME_READER_H__
#include "rcore/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

#include "rcore/c_serial.h"

namespace ncore
{
    class reader_t;

    namespace nserial
    {
        struct frame_sequence_t
        {
            frame_sequence_t(u8 const *sequence, s16 length)
                : mSequence(sequence)
                , mLength(length)
            {
            }
            u8 const *const mSequence;  // Pointer to the byte sequence to match
            const s16       mLength;    // Length of the byte sequence
        };

        struct frame_data_t
        {
            frame_data_t(s16 minFrameLen, s16 maxFrameLen)
                : mStartPtr(nullptr)
                , mEndPtr(nullptr)
                , mMinFrameLen(minFrameLen)
                , mMaxFrameLen(maxFrameLen)
            {
            }

            u8 const *mStartPtr;     // Start frame position
            u8 const *mEndPtr;       // End frame position
            s16       mMinFrameLen;  // Minimum frame length
            s16       mMaxFrameLen;  // Maximum frame length
        };

        struct frame_result_t
        {
            u8 const *frameStart;
            u16       frameLength;
            s8        sequenceIndex;
        };

        struct frame_reader_t
        {
            ncore::reader_t         *mSerialReader;          // Pointer to the serial stream reader
            u8                      *mSerialBuffer;          // Buffer where we are reading serial data into
            u16                      mSerialBufferCapacity;  // Capacity of the serial buffer
            u8                      *mSerialBufferWrite;     // Current write position in the serial buffer
            u8                       mSequenceCount;         // Number of sequences configured
            frame_sequence_t const **mStartSequences;        // Array of possible start sequences
            frame_sequence_t const **mEndSequences;          // Array of possible end sequences (matching the order of start sequences)
            frame_data_t            *mFrameData;             // Active frame data for each detected sequence
            s8                       mFoundSequence;         // Start Sequence that was found
            frame_data_t            *mFoundFrame;            // End Sequence was found, this is the end of that sequence in the serial buffer

            frame_reader_t()
                : mSerialReader(nullptr)
                , mSerialBuffer(nullptr)
                , mSerialBufferCapacity(0)
                , mSerialBufferWrite(nullptr)
                , mSequenceCount(0)
                , mStartSequences(nullptr)
                , mEndSequences(nullptr)
                , mFrameData(nullptr)
                , mFoundSequence(-1)
                , mFoundFrame(nullptr)
            {
            }

            void initialize(reader_t *serial_reader, u8 *buffer, u16 bufferCapacity);
            void set_frame_data(frame_sequence_t const **startSequences, frame_sequence_t const **endSequences, frame_data_t *framePointers, s8 sequenceCount);
            bool read(frame_result_t &result);
        };
    }  // namespace nserial
}  // namespace ncore

#endif  // __ARDUINO_SENSORS_FRAME_READER_H__