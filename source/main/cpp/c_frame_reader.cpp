#include "rcore/c_log.h"
#include "rcore/c_timer.h"
#include "rcore/c_serial.h"

#include "ccore/c_memory.h"
#include "ccore/c_stream.h"

#include "rsensors/c_frame_reader.h"

#ifdef TARGET_ARDUINO
#    include "Arduino.h"
#    include "Wire.h"
#endif

namespace ncore
{
    namespace nserial
    {
        void frame_reader_t::initialize(reader_t *serial_reader, u8 *buffer, u16 bufferCapacity)
        {
            mSerialReader        = serial_reader;
            mSerialBuffer         = buffer;
            mSerialBufferCapacity = bufferCapacity;
            mSerialBufferWrite    = mSerialBuffer;
            mSequenceCount        = 0;
            mStartSequences       = nullptr;
            mFrameData            = nullptr;
            mEndSequences         = nullptr;
            mFoundSequence        = -1;
            mFoundFrame           = nullptr;
        }

        void frame_reader_t::set_frame_data(frame_sequence_t const **startSequences, frame_sequence_t const **endSequences, frame_data_t *frameData, s8 sequenceCount)
        {
            mSequenceCount  = sequenceCount;
            mStartSequences = startSequences;
            mEndSequences   = endSequences;
            mFrameData      = frameData;
            mFoundSequence  = -1;
            mFoundFrame     = nullptr;
            mFrameData      = frameData;

            for (u8 i = 0; i < mSequenceCount; ++i)
            {
                mFrameData[i].mStartPtr = mSerialBuffer;
                mFrameData[i].mEndPtr   = nullptr;
            }
        }

        bool match_sequence(frame_sequence_t const *sequence, u8 const *readCursor)
        {
            u8 j = 0;
            while (j < sequence->mLength)
            {
                if (readCursor[j] != sequence->mSequence[j])
                    break;
                ++j;
            }
            return (j == sequence->mLength);
        }

        bool frame_reader_t::read(frame_result_t &result)
        {
            if (mFoundFrame != nullptr)
            {
                // Last call found a complete frame, so move any remaining data to the front of the buffer
                const u16 remainingDataLen = (u16)(mSerialBufferWrite - mFoundFrame->mEndPtr);
                if (remainingDataLen > 0)
                    g_memmove(mSerialBuffer, mFoundFrame->mEndPtr, remainingDataLen);

                // Update read/write pointer
                mSerialBufferWrite = mSerialBuffer + remainingDataLen;

                // Reset pointers for each sequence
                for (u8 i = 0; i < mSequenceCount; ++i)
                {
                    mFrameData[i].mStartPtr = mSerialBuffer;
                    mFrameData[i].mEndPtr   = nullptr;
                }

                // Reset
                mFoundFrame = nullptr;
            }

            // Read available bytes into buffer using readBytes
            if ((u16)(mSerialBufferWrite - mSerialBuffer) < mSerialBufferCapacity)
            {
                const s64 spaceLeft = mSerialBufferCapacity - (u16)(mSerialBufferWrite - mSerialBuffer);
                mSerialBufferWrite += mSerialReader->read(mSerialBufferWrite, spaceLeft);
            }

            if (mFoundSequence == -1)
            {
                // We step through the serial buffer one byte at a time looking for a start sequence since
                // we could encounter any one of the configured sequences.
                s8 state = 1;
                while (state > 0 && mFoundSequence == -1)
                {
                    state = 0;
                    for (u8 i = 0; i < mSequenceCount; ++i)
                    {
                        frame_sequence_t const *startSequence = mStartSequences[i];
                        if ((mFrameData[i].mStartPtr + startSequence->mLength) <= mSerialBufferWrite)
                        {
                            if (match_sequence(startSequence, mFrameData[i].mStartPtr))
                            {
                                mFrameData[i].mEndPtr = mFrameData[i].mStartPtr + startSequence->mLength;
                                mFoundSequence        = i;
                                break;
                            }
                            mFrameData[i].mStartPtr += 1;
                            state++;
                        }
                    }
                }
                if (mFoundSequence == -1)  // No start sequence found yet
                    return false;
            }

            // Guard for maximum frame size, e.g. we found a header but for some reason the end
            // of a frame never arrives and we keep accumulating data in the buffer.
            if ((mFrameData[mFoundSequence].mEndPtr - mFrameData[mFoundSequence].mStartPtr) > mFrameData[mFoundSequence].mMaxFrameLen)
            {
                // Discard current search, we have a start sequence but exceeded max frame size without finding end sequence
                mFoundSequence     = -1;
                mSerialBufferWrite = mSerialBuffer;
                for (u8 i = 0; i < mSequenceCount; ++i)
                {
                    mFrameData[i].mStartPtr = mSerialBuffer;
                    mFrameData[i].mEndPtr   = nullptr;
                }
                return false;
            }

            // Search for 'end sequence' that matches the found start sequence
            frame_sequence_t const *endSequence = mEndSequences[mFoundSequence];
            while (true)
            {
                if ((mFrameData[mFoundSequence].mEndPtr + endSequence->mLength) > mSerialBufferWrite)
                    return false;
                if (match_sequence(endSequence, mFrameData[mFoundSequence].mEndPtr))
                {  // End sequence found, set found end pointer to after the end sequence
                    mFrameData[mFoundSequence].mEndPtr += endSequence->mLength;
                    break;
                }
                if ((mFrameData[mFoundSequence].mEndPtr + endSequence->mLength) >= mSerialBufferWrite)
                    return false;
                mFrameData[mFoundSequence].mEndPtr += 1;
            }

            // Setup found message
            mFoundFrame          = &mFrameData[mFoundSequence];
            result.frameLength   = (u16)(mFoundFrame->mEndPtr - mFoundFrame->mStartPtr);
            result.frameStart    = mFoundFrame->mStartPtr;
            result.sequenceIndex = mFoundSequence;

            // Initialize to find the next sequence
            mFoundSequence = -1;

            return true;
        }
    }  // namespace nserial
}  // namespace ncore