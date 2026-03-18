#include "rsensors/c_frame_reader.h"
#include "lib_rd03d_v1/c_rd03d.h"

#include "rcore/c_log.h"
#include "rcore/c_timer.h"
#include "rcore/c_serial.h"

#include "ccore/c_endian.h"
#include "ccore/c_memory.h"
#include "ccore/c_stream.h"

#ifdef TARGET_ARDUINO
#    include "Arduino.h"
#    include "Wire.h"
#endif

namespace ncore
{
    namespace nsensors
    {
        namespace nrd03d
        {
            static void LOG_ERR(const char *msg) { ncore::nlog::error(msg); }
            static void LOG_ERRF(const char *fmt, ...) {}

#define EPERM 1
#define EINVAL 22
#define ENOTSUP 95

#define RD03D_TX_BUF_MAX_LEN 18
#define RD03D_RX_BUF_MAX_LEN 64
#define RD03D_UART_BAUD_RATE 256000
#define RD03D_MAX_TARGETS    3

            enum rd03d_protocol_cmd_idx
            {
                RD03D_CMD_IDX_OPEN_CMD_MODE,
                RD03D_CMD_IDX_CLOSE_CMD_MODE,
                RD03D_CMD_IDX_DEBUGGING_MODE,
                RD03D_CMD_IDX_REPORTING_MODE,
                RD03D_CMD_IDX_RUNNING_MODE,

                RD03D_CMD_IDX_SET_MIN_DISTANCE,
                RD03D_CMD_IDX_SET_MAX_DISTANCE,
                RD03D_CMD_IDX_SET_MIN_FRAMES,  // Min number of frames considered as appearing
                RD03D_CMD_IDX_SET_MAX_FRAMES,  // Max number of frames considered as dissapearing
                RD03D_CMD_IDX_SET_DELAY_TIME,  // The delay time for considering as dissapeared

                RD03D_CMD_IDX_GET_MIN_DISTANCE,
                RD03D_CMD_IDX_GET_MAX_DISTANCE,
                RD03D_CMD_IDX_GET_MIN_FRAMES,
                RD03D_CMD_IDX_GET_MAX_FRAMES,
                RD03D_CMD_IDX_GET_DELAY_TIME,

                RD03D_CMD_IDX_SINGLE_TARGET_MODE,  // Detection mode single target
                RD03D_CMD_IDX_MULTI_TARGET_MODE,   // Detection mode multiple targets
            };

            enum rd03d_detection_mode
            {
                RD03D_DETECTION_MODE_SINGLE_TARGET,
                RD03D_DETECTION_MODE_MULTI_TARGET,
            };

            enum rd03d_operation_mode
            {
                RD03D_OPERATION_MODE_CMD    = 0x80,
                RD03D_OPERATION_MODE_DEBUG  = 0x00,
                RD03D_OPERATION_MODE_REPORT = 0x01,
                RD03D_OPERATION_MODE_RUN    = 0x02,
            };

            enum rd03d_property
            {
                RD03D_PROP_MIN_DISTANCE,
                RD03D_PROP_MAX_DISTANCE,
                RD03D_PROP_MIN_FRAMES,
                RD03D_PROP_MAX_FRAMES,
                RD03D_PROP_DELAY_TIME,
                RD03D_PROP_DETECTION_MODE,
                RD03D_PROP_OPERATION_MODE,
            };

            struct rd03d_data_t
            {
                u8 tx_bytes;    /* Number of bytes send so far */
                u8 tx_data_len; /* Number of bytes to send */
                u8 tx_data[RD03D_TX_BUF_MAX_LEN];

                u8 rx_enabled;     /* Flag to indicate if RX is enabled */
                u8 rx_bytes;       /* Number of bytes received so far */
                u8 rx_frame_start; /* Start of an ACK or Report in the buffer */
                u8 rx_data_len;    /* Number of bytes to receive */
                u8 rx_data[RD03D_RX_BUF_MAX_LEN];

                u8 operation_mode;
                u8 detection_mode;

                target_t targets[RD03D_MAX_TARGETS];

                u16 min_distance;
                u16 max_distance;
                u16 min_frames;
                u16 max_frames;
                u16 delay_time;
            };

            rd03d_data_t data;

            struct rd03d_cfg_t
            {
                u16 min_distance;
                u16 max_distance;
                u16 min_frames;
                u16 max_frames;
                u16 delay_time;
            };

            rd03d_cfg_t config;

            struct device_t
            {
                rd03d_cfg_t            *config;
                rd03d_data_t           *data;
                writer_t               *serial_writer;
                nserial::frame_reader_t frame_reader;
            };

            device_t device = {&config, &data, nullptr};

            // Endianness is Little Endian

            static bool rd03d_rx_frame(device_t *dev);
            static void rd03d_tx_data(device_t *dev);

            // clang-format off

            // Protocol data frame format, head and tail
            static const u8 RD03D_CMD_HEAD[]    = {0xFD, 0xFC, 0xFB, 0xFA};
            static const u8 RD03D_CMD_TAIL[]    = {0x04, 0x03, 0x02, 0x01};
            static const u8 RD03D_FRAME_HEAD[]  = {0xF4, 0xF3, 0xF2, 0xF1};
            static const u8 RD03D_FRAME_TAIL[]  = {0xF8, 0xF7, 0xF6, 0xF5};
            static const u8 RD03D_REPORT_HEAD[] = {0xAA, 0xFF, 0x03, 0x00};
            static const u8 RD03D_REPORT_TAIL[] = {0x55, 0xCC};

            // Note: So in the comments you may read Word which means that this is a register value. 
            //       The byte order of the command is thus swapped compared to an indicated Word value.

            // Send command protocol frame format
            // |----------------------------------------------------------------------------------
            // | Frame header | Intra-frame data length  |  Intra-frame data  |   End of frame   |
            // | FD FC FB FA  |  2 bytes                 |    See table 4     |   04 03 02 01    |
            // |----------------------------------------------------------------------------------

            // Table 4 
            // Send intra-frame data format
            // |----------------------------------------------------------
            // |  Command Word (2 bytes)   |    Command value (N bytes)  |
            // |----------------------------------------------------------

            // 	RD03D_CMD_IDX_OPEN_CMD_MODE      = { RD03D_CMD_HEADER_BEGIN, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_CLOSE_CMD_MODE     = { RD03D_CMD_HEADER_BEGIN, 0x02, 0x00, 0xFE, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_DEBUGGING_MODE     = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_REPORTING_MODE     = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_RUNNING_MODE       = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_SET_MIN_DISTANCE   = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_SET_MAX_DISTANCE   = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END },
            // 	RD03D_CMD_IDX_SET_MIN_FRAMES     = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_SET_MAX_FRAMES     = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_SET_DELAY_TIME     = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x07, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_GET_MIN_DISTANCE   = { RD03D_CMD_HEADER_BEGIN, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_GET_MAX_DISTANCE   = { RD03D_CMD_HEADER_BEGIN, 0x04, 0x00, 0x08, 0x00, 0x01, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_GET_MIN_FRAMES     = { RD03D_CMD_HEADER_BEGIN, 0x04, 0x00, 0x08, 0x00, 0x02, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_GET_MAX_FRAMES     = { RD03D_CMD_HEADER_BEGIN, 0x04, 0x00, 0x08, 0x00, 0x03, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_GET_DELAY_TIME     = { RD03D_CMD_HEADER_BEGIN, 0x04, 0x00, 0x08, 0x00, 0x04, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_SINGLE_TARGET_MODE = { RD03D_CMD_HEADER_BEGIN, 0x02, 0x00, 0x80, 0x00, RD03D_CMD_HEADER_END }
            // 	RD03D_CMD_IDX_MULTI_TARGET_MODE  = { RD03D_CMD_HEADER_BEGIN, 0x02, 0x00, 0x90, 0x00, RD03D_CMD_HEADER_END },

            // return the length of the command
            static s32 rd03d_prepare_cmd(enum rd03d_protocol_cmd_idx cmd_idx, u8 *cmd_buffer, s32 value) {
                // Assume the header is already set
                s32 l = 4; // Skip header

                if (cmd_idx  == RD03D_CMD_IDX_OPEN_CMD_MODE) 
                {
                    cmd_buffer[l++] = 0x04; // Intra-frame data length
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0xFF; // Command word
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0x01; // 
                    cmd_buffer[l++] = 0x00; //
                }
                else if (cmd_idx == RD03D_CMD_IDX_CLOSE_CMD_MODE) 
                {
                    cmd_buffer[l++] = 0x02; // Intra-frame data length
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0xFE; // Command word
                    cmd_buffer[l++] = 0x00; //
                } 
                else if (cmd_idx >= RD03D_CMD_IDX_DEBUGGING_MODE && cmd_idx <= RD03D_CMD_IDX_RUNNING_MODE) 
                {
                    cmd_buffer[l++] = 0x08; // Intra-frame data length
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0x12; // Command word
                    cmd_buffer[l++] = 0x00; //
                    // 6 bytes of data
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0x00; //

                    if (cmd_idx == RD03D_CMD_IDX_REPORTING_MODE) {
                        cmd_buffer[l++] = 0x04; // Reporting mode
                    } else if (cmd_idx == RD03D_CMD_IDX_RUNNING_MODE) {
                        cmd_buffer[l++] = 0x64; // Running mode
                    } else {
                        cmd_buffer[l++] = 0x00; // Debugging mode			
                    }

                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0x00; //
                }
                else if (cmd_idx >= RD03D_CMD_IDX_SET_MIN_DISTANCE && cmd_idx <= RD03D_CMD_IDX_SET_DELAY_TIME) 
                {
                    cmd_buffer[l++] = 0x08; // Intra-frame data length
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0x07; // Command word
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = cmd_idx - RD03D_CMD_IDX_SET_MIN_DISTANCE; // Command value
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = value & 0xFF; // Set value, 32-bit
                    cmd_buffer[l++] = (value >> 8) & 0xFF;
                    cmd_buffer[l++] = (value >> 16) & 0xFF;
                    cmd_buffer[l++] = (value >> 24) & 0xFF;
                }
                else if (cmd_idx >= RD03D_CMD_IDX_GET_MIN_DISTANCE && cmd_idx <= RD03D_CMD_IDX_GET_DELAY_TIME) 
                {
                    cmd_buffer[l++] = 0x04; // Intra-frame data length
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = 0x08; // Command word
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = cmd_idx - RD03D_CMD_IDX_GET_MIN_DISTANCE; // Command value
                    cmd_buffer[l++] = 0x00; //
                }
                else if (cmd_idx >= RD03D_CMD_IDX_SINGLE_TARGET_MODE && cmd_idx <= RD03D_CMD_IDX_MULTI_TARGET_MODE) 
                {
                    cmd_buffer[l++] = 0x02; // Intra-frame data length
                    cmd_buffer[l++] = 0x00; //
                    cmd_buffer[l++] = RD03D_CMD_IDX_SINGLE_TARGET_MODE ? 0x80 : 0x90; // Command word
                    cmd_buffer[l++] = 0x00; //
                }

                // Write the tail
                cmd_buffer[l++] = RD03D_CMD_TAIL[0];
                cmd_buffer[l++] = RD03D_CMD_TAIL[1];
                cmd_buffer[l++] = RD03D_CMD_TAIL[2];
                cmd_buffer[l++] = RD03D_CMD_TAIL[3];

                return l;
            }

            // ACK command protocol frame format
            // |--------------------------------------------------------------------------------
            // | Frame header  | Intra-frame data length  |  Intra-frame data  |  End of frame |
            // | FD FC FB FA   |       2bytes             |      See table 6   |   04 03 02 01 |
            // |--------------------------------------------------------------------------------

            // Table 6
            // ACK intra-frame data format
            // |------------------------------------------------------------------
            // |  Send Command Word | 0x0100 (2 bytes)  | Return value (N bytes)  |
            // |------------------------------------------------------------------

            // static const u8 RD03D_CMD_ACK_IDX_OPEN_CMD_MODE[]   = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x01, 0x00, 0x40, 0x00, RD03D_CMD_HEADER_END };
            // static const u8 RD03D_CMD_ACK_IDX_CLOSE_CMD_MODE[]  = { RD03D_CMD_HEADER_BEGIN, 0x04, 0x00, 0xFE, 0x01, 0x00, 0x00, RD03D_CMD_HEADER_END };

            // Protocol, ACKS we get for set and get commands, the ACK related to the get cmd contains a 4 byte variable at (rx_buffer[10] to rx_buffer[13])
            // static const u8 RD03D_CMD_ACK_IDX_SET_XXX[] = { RD03D_CMD_HEADER_BEGIN, 0x04, 0x00, 0x07, 0x01, 0x00, 0x00, RD03D_CMD_HEADER_END },
            // static const u8 RD03D_CMD_ACK_IDX_GET_XXX[] = { RD03D_CMD_HEADER_BEGIN, 0x08, 0x00, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, RD03D_CMD_HEADER_END },

            // clang-format on
            static inline bool data_frame_has_valid_len(s32 data_len) { return (data_len == 12 || data_len == 14 || data_len == 18); }
            static inline bool data_frame_has_valid_head(const u8 *data, s32 data_len) { return data[0] == RD03D_FRAME_HEAD[0] && data[1] == RD03D_FRAME_HEAD[1] && data[2] == RD03D_FRAME_HEAD[2] && data[3] == RD03D_FRAME_HEAD[3]; }

            static inline s32 data_frame_get_intra_frame_data_len(const u8 *data, s32 data_len)
            {
                const s32 len = data[4] | (data[5] << 8);
                return (len != 2 && len != 4 && len != 8) ? -1 : len;
            }

            static inline bool data_frame_has_valid_tail(const u8 *data, s32 data_len, s32 intra_frame_data_len)
            {
                if (data_len != (4 + 2 + intra_frame_data_len + 4))
                {
                    return false;
                }

                return data[4 + 2 + intra_frame_data_len + 0] == RD03D_FRAME_TAIL[0] && data[4 + 2 + intra_frame_data_len + 1] == RD03D_FRAME_TAIL[1] && data[4 + 2 + intra_frame_data_len + 2] == RD03D_FRAME_TAIL[2] &&
                       data[4 + 2 + intra_frame_data_len + 3] == RD03D_FRAME_TAIL[3];
            }

            static s32 verify_ack_for_open_cmd(const u8 *data, s32 data_len)
            {
                const s32 intra_frame_data_len = 8;
                if (data_len != 4 + 2 + intra_frame_data_len + 4)
                {
                    return -1;
                }

                if (data_frame_has_valid_head(data, data_len) == false)
                {
                    return -1;
                }
                if (data_frame_get_intra_frame_data_len(data, data_len) != intra_frame_data_len)
                {
                    return -1;
                }

                if (data[6] == 0xFF && data[7] == 0x01 && data[8] == 0x00 && data[9] == 0x00 && data[10] == 0x01 && data[11] == 0x00 && data[12] == 0x40 && data[13] == 0x00)
                {
                    return data_frame_has_valid_tail(data, data_len, intra_frame_data_len) ? 0 : -1;
                }
                return -1;
            }

            static s32 verify_ack_for_close_cmd(const u8 *data, s32 data_len)
            {
                const s32 intra_frame_data_len = 4;
                if (data_len != 4 + 2 + intra_frame_data_len + 4)
                {
                    return -1;
                }

                if (data_frame_has_valid_head(data, data_len) == false)
                {
                    return -1;
                }
                if (data_frame_get_intra_frame_data_len(data, data_len) != intra_frame_data_len)
                {
                    return -1;
                }

                if (data[6] == 0xFE && data[7] == 0x01 && data[8] == 0x00 && data[9] == 0x00)
                {
                    return data_frame_has_valid_tail(data, data_len, intra_frame_data_len) ? 0 : -1;
                }
                return -1;
            }

            static s32 verify_ack_for_set_cmd(const u8 *data, s32 data_len)
            {
                const s32 intra_frame_data_len = 4;
                if (data_len != 4 + 2 + intra_frame_data_len + 4)
                {
                    return -1;
                }

                if (data_frame_has_valid_head(data, data_len) == false)
                {
                    return -1;
                }
                if (data_frame_get_intra_frame_data_len(data, data_len) != intra_frame_data_len)
                {
                    return -1;
                }

                if (data[6] == 0x07 && data[7] == 0x01 && data[8] == 0x00 && data[9] == 0x00)
                {
                    return data_frame_has_valid_tail(data, data_len, intra_frame_data_len) ? 0 : -1;
                }
                return -1;
            }

            static s32 verify_ack_for_get_cmd(const u8 *data, s32 data_len, s32 *value)
            {
                *value = 0;

                const s32 intra_frame_data_len = 8;
                if (data_len != 4 + 2 + intra_frame_data_len + 4)
                {
                    return -1;
                }

                if (data_frame_has_valid_head(data, data_len) == false)
                {
                    return -1;
                }
                if (data_frame_get_intra_frame_data_len(data, data_len) != intra_frame_data_len)
                {
                    return -1;
                }

                if (data[6] == 0x08 && data[7] == 0x01 && data[8] == 0x00 && data[9] == 0x00)
                {
                    *value = ncore::nendian_le::read_u32(data + 10);
                    return data_frame_has_valid_tail(data, data_len, intra_frame_data_len) ? 0 : -1;
                }
                return -1;
            }

            static void rd03d_uart_flush(device_t *dev)
            {
                // while (dev->frame_reader.read() >= 0)
                // {
                //     // Discard data
                // }
            }

#define RD03D_TX   2
#define RD03D_RX   1
#define RD03D_TXRX (RD03D_TX | RD03D_RX)

            static s32 rd03d_send_cmd(device_t *dev, rd03d_protocol_cmd_idx cmd_idx, u8 txrx, s32 value)
            {
                rd03d_data_t      *data = dev->data;
                const rd03d_cfg_t *cfg  = dev->config;
                s32                ret;

                if (data->operation_mode != RD03D_OPERATION_MODE_CMD)
                {
                    return -EPERM;
                }

                data->tx_data_len = rd03d_prepare_cmd(cmd_idx, data->tx_data, value);

                if (txrx & RD03D_TX)
                {
                    // Transmit the command
                    rd03d_tx_data(dev);
                }

                if (txrx & RD03D_RX)
                {
                    // Receive the response
                    rd03d_rx_frame(dev);
                }

                return ret;
            }

            static s32 rd03d_open_cmd_mode(device_t *dev)
            {
                struct rd03d_data_t      *data = dev->data;
                const struct rd03d_cfg_t *cfg  = dev->config;
                s32                       ret;

                /*
                Note in the documentation:
                   (1) Send "Open command mode" (because the chip may still output
                       data, the data received by the frame_reader port will contain
                       waveform data)
                   (2) Empty frame_reader port cache data (generally delay for around 100ms,
                       to ensure that frame_reader port data is empty)
                   (3) Send the Open Command Mode, once again, and analyze returned
                       results.
                */

                // Open command mode
                ret = rd03d_send_cmd(dev, RD03D_CMD_IDX_OPEN_CMD_MODE, RD03D_TX, 1);
                if (ret < 0)
                {
                    return ret;
                }

                // Wait for 100ms to ensure that the frame_reader port data is not
                // receiving anymore report data
                ntimer::delay(100);

                // Flush the frame_reader port cache
                rd03d_uart_flush(dev);

                // Open command mode again
                ret = rd03d_send_cmd(dev, RD03D_CMD_IDX_OPEN_CMD_MODE, RD03D_TXRX, 1);
                if (ret < 0)
                {
                    return ret;
                }

                // Verify the command was acknowledged successfully
                ret = verify_ack_for_open_cmd(data->rx_data, data->rx_data_len);
                return ret;
            }

            static s32 rd03d_close_cmd_mode(device_t *dev)
            {
                struct rd03d_data_t *data = dev->data;
                s32                  ret;

                // Close command mode
                ret = rd03d_send_cmd(dev, RD03D_CMD_IDX_CLOSE_CMD_MODE, RD03D_TXRX, 1);
                if (ret < 0)
                {
                    return ret;
                }

                // Verify the command was acknowledged successfully
                ret = verify_ack_for_close_cmd(data->rx_data, data->rx_data_len);
                return ret;
            }

            static s32 rd03d_set_attribute(device_t *dev, enum rd03d_protocol_cmd_idx cmd_idx, s32 value)
            {
                struct rd03d_data_t *data = dev->data;

                s32 ret;

                // This is always a mutable command, so we need to copy the command into
                // the transmit (tx) buffer, set the value, and then send the command.

                // Note: The user has to explicitly set the command mode to be able to
                //       set and get attributes/channel data.
                if ((data->operation_mode & RD03D_OPERATION_MODE_CMD) == 0)
                {
                    ret = rd03d_open_cmd_mode(dev);
                    if (ret < 0)
                    {
                        LOG_ERR("Error, open command mode failed");
                        return -EINVAL;
                    }
                    data->operation_mode |= RD03D_OPERATION_MODE_CMD;
                }

                // Set the attribute value in the command
                ret = rd03d_send_cmd(dev, cmd_idx, RD03D_TXRX, value);
                if (ret < 0)
                {
                    LOG_ERRF("Error, set attribute command (%d) failed", cmd_idx);
                    return ret;
                }

                // Verify the command was acknowledged successfully
                ret = verify_ack_for_set_cmd(data->rx_data, data->rx_data_len);
                if (ret < 0)
                {
                    LOG_ERRF("Error, set attribute command (%d) did not get valid ACK", cmd_idx);
                    return ret;
                }

                // Note: When setting RD03D_ATTR_OPERATION_MODE to any of the reporting
                //       modes, the sensor will close the command mode.
                //       This means that the command mode will be closed automatically
                //       and the sensor will enter the reporting mode.
                if ((cmd_idx >= RD03D_CMD_IDX_DEBUGGING_MODE && cmd_idx <= RD03D_CMD_IDX_RUNNING_MODE))
                {
                    if ((data->operation_mode & RD03D_OPERATION_MODE_CMD) == RD03D_OPERATION_MODE_CMD)
                    {
                        ret = rd03d_close_cmd_mode(dev);
                        if (ret < 0)
                        {
                            LOG_ERR("Error, close command mode failed");
                            return -EINVAL;
                        }
                        data->operation_mode &= ~RD03D_OPERATION_MODE_CMD;
                    }
                }

                return ret;
            }

            static inline s32 rd03d_get_attribute(device_t *dev, enum rd03d_protocol_cmd_idx cmd_idx, s32 *value)
            {
                struct rd03d_data_t *data = dev->data;

                s32 ret;

                // This is always a mutable command, so we need to copy the command into
                // the transmit (tx) buffer, set the value, and then send the command.

                // Note: The user has to explicitly set the command mode to be able to
                //       set and get attributes/channel data.
                if ((data->operation_mode & RD03D_OPERATION_MODE_CMD) == 0)
                {
                    ret = rd03d_open_cmd_mode(dev);
                    if (ret < 0)
                    {
                        LOG_ERR("Error, open command mode failed");
                        return -EINVAL;
                    }
                    data->operation_mode |= RD03D_OPERATION_MODE_CMD;
                }

                // Get command
                ret = rd03d_send_cmd(dev, cmd_idx, RD03D_TXRX, 0);
                if (ret < 0)
                {
                    LOG_ERRF("Error, get attribute command (%d) failed", (s32)cmd_idx);
                    return ret;
                }

                // Read the value from the response
                ret = verify_ack_for_get_cmd(data->rx_data, data->rx_data_len, value);
                if (ret < 0)
                {
                    LOG_ERRF("Error, get attribute command (%d) did not get valid ACK", cmd_idx);
                    return ret;
                }

                // Note: When setting RD03D_ATTR_OPERATION_MODE to any of the reporting
                //       modes, the sensor will close the command mode.
                //       This means that the command mode will be closed automatically
                //       and the sensor will enter the reporting mode.
                if ((cmd_idx >= RD03D_CMD_IDX_DEBUGGING_MODE && cmd_idx <= RD03D_CMD_IDX_RUNNING_MODE))
                {
                    if ((data->operation_mode & RD03D_OPERATION_MODE_CMD) == RD03D_OPERATION_MODE_CMD)
                    {
                        ret = rd03d_close_cmd_mode(dev);
                        if (ret < 0)
                        {
                            return -EINVAL;
                        }
                        data->operation_mode &= ~RD03D_OPERATION_MODE_CMD;
                    }
                }

                return ret;
            }

            static s32 rd03d_sample_fetch(device_t *dev)
            {
                // When in 'reporting' mode, the sensor will send 'reports' continuously
                // and data will become available in the RX buffer.
                rd03d_data_t      *data = dev->data;
                const rd03d_cfg_t *cfg  = dev->config;

                s32 ret;

                // We decode the rx buffer into data->targets
                if (data->operation_mode == RD03D_OPERATION_MODE_REPORT)
                {
                    if (rd03d_rx_frame(dev))
                    {
                        u8 const *rx = data->rx_data;

                        // TODO actually the verification has already been done when receiving, we
                        //      should not really have to verify again!
#ifdef RD03D_DEBUG
                        const s32 report_len = 8 * RD03D_MAX_TARGETS;
                        if (data->rx_data_len != sizeof(RD03D_REPORT_HEAD) + report_len + sizeof(RD03D_REPORT_TAIL))
                        {
                            LOG_ERR("Invalid report frame, data frame length mismatch");
                            return -EINVAL;
                        }

                        if (rx[0] != RD03D_REPORT_HEAD[0] || rx[1] != RD03D_REPORT_HEAD[1] || rx[2] != RD03D_REPORT_HEAD[2] || rx[3] != RD03D_REPORT_HEAD[3])
                        {
                            LOG_ERR("Invalid report frame, head mismatch");
                            return -EINVAL;
                        }

                        if (rx[28] != RD03D_REPORT_TAIL[0] || rx[29] != RD03D_REPORT_TAIL[1])
                        {
                            LOG_ERR("Invalid report frame, tail mismatch");
                            return -EINVAL;
                        }
#endif
                        // Decode the response
                        // RD03D_REPORT_HEAD
                        //   Target 1 { x, y, speed, distance }
                        //   Target 2 { x, y, speed, distance }
                        //   Target 3 { x, y, speed, distance }
                        // RD03D_REPORT_TAIL

                        // For decoding targets, assume each target occupies 8 bytes and parsing them
                        // sequentially. Here we first skip the header and move through the rx buffer.
                        s32 ti = 0;
                        for (s32 i = 4; i < (data->rx_bytes - 2) && ti < RD03D_MAX_TARGETS; i += 8)
                        {
                            data->targets[ti].x = (s16)(rx[i] | (rx[i + 1] << 8)) - 0x200;
                            data->targets[ti].y = (s16)(rx[i + 2] | (rx[i + 3] << 8)) - 0x8000;
                            data->targets[ti].v = (s16)(rx[i + 4] | (rx[i + 5] << 8)) - 0x10;
                            // data->targets[ti].distance = (uint16_t)(rx[i + 6] | (rx[i + 7] << 8));
                        }
                    }
                }

                return -ENOTSUP;
            }

            static void rd03d_tx_data(device_t *dev)
            {
                rd03d_data_t *data = dev->data;
                data->rx_bytes     = 0;

                while (true)
                {
                    data->tx_bytes += dev->serial_writer->write(&data->tx_data[data->tx_bytes], data->tx_data_len - data->tx_bytes);
                    if (data->tx_bytes == data->tx_data_len)
                    {
                        data->tx_bytes = 0;
                        break;
                    }
                }
            }

            static s32 rd03d_init(device_t *dev)
            {
                rd03d_data_t      *data = dev->data;
                const rd03d_cfg_t *cfg  = dev->config;

                s32 ret = 0;

                data->tx_bytes    = 0;
                data->tx_data_len = 0;
                g_memset(data->tx_data, 0, RD03D_TX_BUF_MAX_LEN);
                data->tx_data[0] = RD03D_CMD_HEAD[0];
                data->tx_data[1] = RD03D_CMD_HEAD[1];
                data->tx_data[2] = RD03D_CMD_HEAD[2];
                data->tx_data[3] = RD03D_CMD_HEAD[3];

                /* Default operation mode when turning on the device */
                data->operation_mode = RD03D_OPERATION_MODE_REPORT;
                data->detection_mode = RD03D_DETECTION_MODE_MULTI_TARGET;

                for (s32 i = 0; i < RD03D_MAX_TARGETS; i++)
                {
                    data->targets[i].x = 0;
                    data->targets[i].y = 0;
                    data->targets[i].v = 0;
                }

                rd03d_uart_flush(dev);

                // Set configured attributes
                if (cfg->min_distance != 0xffff)
                {
                    ret = rd03d_set_attribute(dev, RD03D_CMD_IDX_SET_MIN_DISTANCE, cfg->min_distance);
                }
                if (cfg->max_distance != 0xffff)
                {
                    ret = rd03d_set_attribute(dev, RD03D_CMD_IDX_SET_MAX_DISTANCE, cfg->max_distance);
                }
                if (cfg->min_frames != 0xffff)
                {
                    ret = rd03d_set_attribute(dev, RD03D_CMD_IDX_SET_MIN_FRAMES, cfg->min_frames);
                }
                if (cfg->max_frames != 0xffff)
                {
                    ret = rd03d_set_attribute(dev, RD03D_CMD_IDX_SET_MAX_FRAMES, cfg->max_frames);
                }
                if (cfg->delay_time != 0xffff)
                {
                    ret = rd03d_set_attribute(dev, RD03D_CMD_IDX_SET_DELAY_TIME, cfg->delay_time);
                }

                /* Activate report mode */
                ret = rd03d_set_attribute(dev, RD03D_CMD_IDX_REPORTING_MODE, 0);

                return ret;
            }

        }  // namespace nrd03d
    }  // namespace nsensors
}  // namespace ncore
