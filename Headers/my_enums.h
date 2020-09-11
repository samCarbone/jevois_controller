#ifndef MY_ENUMS_H
#define MY_ENUMS_H

enum source_t : char { SRC_PC, SRC_JV, SRC_ESP, SRC_FC };
enum dest_t : char { DST_PC, DST_JV, DST_ESP, DST_FC };
enum mids_t : char { MID_PING, MID_MODE, MID_LAND, MID_QUIT, MID_MSP, MID_ALT, MID_JSON };
enum resp_t : char { RSP_FALSE, RSP_TRUE };
enum esp_modes_t : char { ESP_MODE_ERR, ESP_MODE_PC, ESP_MODE_JV };
enum jv_ctrl_t : char { JV_CTRL_ERR, JV_CTRL_ENA, JV_CTRL_DIS };
enum jv_land_t : char { JV_LAND_ERR, JV_LAND_ENA, JV_LAND_DIS };
enum jv_quit_t : char { JV_QUIT_FALSE, JV_QUIT_TRUE };
enum log_levels { LL_ALL, LL_DEBUG, LL_INFO, LL_WARN, LL_ERROR, LL_FATAL, LL_OFF };

#endif // MY_ENUMS_H
