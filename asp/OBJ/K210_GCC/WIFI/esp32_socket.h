/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2020-2021 by fukuen
 *
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: esp32_socket.h 2416 2020-12-10 08:06:20Z fukuen $
 */

/*
 *  ESP32_SOCKET プログラムのヘッダファイル
 */

/*
 *  ターゲット依存の定義
 */
#include "kernel_cfg.h"
#include <target_syssvc.h>
#include <stdint.h>
#include "device.h"
#include "i2c.h"
#include "spi.h"

#define ESP32_DEBUG                     (0)

#define ESP32_ADDRESS                   (0x04)

#if ESP32_DEBUG == 1
#define PRINTF1(fmt)        printf(fmt)
#define PRINTF11(fmt, ...)  printf(fmt, __VA_ARGS__)
#define PRINTF2(fmt)
#define PRINTF21(fmt, ...)
#define PRINTF3(fmt)
#define PRINTF31(fmt, ...)
#elif ESP32_DEBUG == 2
#define PRINTF1(fmt)        printf(fmt)
#define PRINTF11(fmt, ...)  printf(fmt, __VA_ARGS__)
#define PRINTF2(fmt)        printf(fmt)
#define PRINTF21(fmt, ...)  printf(fmt, __VA_ARGS__)
#define PRINTF3(fmt, ...)
#define PRINTF31(fmt, ...)
#elif ESP32_DEBUG >= 3
#define PRINTF1(fmt)        printf(fmt)
#define PRINTF11(fmt, ...)  printf(fmt, __VA_ARGS__)
#define PRINTF2(fmt)        printf(fmt)
#define PRINTF21(fmt, ...)  printf(fmt, __VA_ARGS__)
#define PRINTF3(fmt)        printf(fmt)
#define PRINTF31(fmt, ...)  printf(fmt, __VA_ARGS__)
#else
#define PRINTF1(fmt)
#define PRINTF11(fmt, ...)
#define PRINTF2(fmt)
#define PRINTF21(fmt, ...)
#define PRINTF3(fmt)
#define PRINTF31(fmt, ...)
#endif

typedef enum
{
	SET_NET_CMD                 = (0x10),
	SET_PASSPHRASE_CMD          = (0x11),
/*
	SET_KEY_CMD                 = (0x12), // add
	SET_IP_CONFIG_CMD           = (0x14), // add
	SET_DNS_CONFIG_CMD          = (0x15), // add
	SET_HOSTNAME_CMD            = (0x16), // add
	SET_POWER_MODE_CMD          = (0x17), // add
	SET_AP_NET_CMD              = (0x18), // add _
	SET_AP_PASS_PHRASE_CMD      = (0x19), // add _
*/
	SET_DEBUG_CMD               = (0x1A),
/*
	GET_TEMPERATURE_CMD         = (0x1B), // add
*/
	GET_CONN_STATUS_CMD         = (0x20),
/*
	GET_IPADDR_CMD              = (0x21),
	GET_MACADDR_CMD             = (0x22),
	GET_CURR_SSID_CMD           = (0x23),
	GET_CURR_BSSID_CMD          = (0x24), // add _
	GET_CURR_RSSI_CMD           = (0x25),
	GET_CURR_ENCT_CMD           = (0x26),
	SCAN_NETWORKS               = (0x27),
	START_SERVER_TCP_CMD        = (0x28), // add _
*/
	GET_STATE_TCP_CMD           = (0x29),
	DATA_SENT_TCP_CMD           = (0x2A),
	AVAIL_DATA_TCP_CMD          = (0x2B),
	GET_DATA_TCP_CMD            = (0x2C),
	START_CLIENT_TCP_CMD        = (0x2D),
	STOP_CLIENT_TCP_CMD         = (0x2E),
	GET_CLIENT_STATE_TCP_CMD    = (0x2F),
	DISCONNECT_CMD              = (0x30),
/*
	GET_IDX_RSSI_CMD            = (0x32),
	GET_IDX_ENCT_CMD            = (0x33),
*/
	REQ_HOST_BY_NAME_CMD        = (0x34),
	GET_HOST_BY_NAME_CMD        = (0x35),
/*
	START_SCAN_NETWORKS         = (0x36),
*/
	GET_FW_VERSION_CMD          = (0x37),
/*
	SEND_UDP_DATA_CMD           = (0x39), // START_CLIENT_TCP_CMD set ip,port, then ADD_UDP_DATA_CMD to add data then SEND_UDP_DATA_CMD to call sendto
	GET_REMOTE_INFO_CMD         = (0x3A),
*/
	GET_TIME_CMD                = (0x3B),
/*
	GET_IDX_BSSID_CMD           = (0x3C), // add
	GET_IDX_CHANNEL_CMD         = (0x3D), // add
	PING_CMD                    = (0x3E),
*/
	GET_SOCKET_CMD              = (0x3F),
	SET_CLIENT_CERT_CMD         = (0x40),
	SET_CERT_KEY_CMD            = (0x41),
	SEND_DATA_TCP_CMD           = (0x44),
	GET_DATABUF_TCP_CMD         = (0x45),
/*
	ADD_UDP_DATA_CMD            = (0x46),
	WPA2_ENT_SET_IDENTITY_CMD   = (0x4A), // add
	WPA2_ENT_SET_USERNAME_CMD   = (0x4B), // add
	WPA2_ENT_SET_PASSWORD_CMD   = (0x4C), // add
	WPA2_ENT_SET_CACERT_CMD     = (0x4D), // add
	WPA2_ENT_SET_CERT_KEY_CMD   = (0x4E), // add
	WPA2_ENT_ENABLE_CMD         = (0x4F), // add
	SET_PIN_MODE_CMD            = (0x50), // add
	SET_DIGITAL_WRITE_CMD       = (0x51), // add
	SET_ANALOG_WRITE_CMD        = (0x52), // add
	GET_ADC_VAL_CMD             = (0x53),
*/
	SOFT_RESET_CMD              = (0x54),
	DUMMY_CMD                   = (0x55),
	START_CMD                   = (0xE0),
	END_CMD                     = (0xEE),
	ERR_CMD                     = (0xEF)
} esp32_cmd_enum_t;

typedef enum {
	CMD_FLAG                    = (0),
	REPLY_FLAG                  = (1<<7)
} esp32_flag_t;

typedef enum{
	SOCKET_CLOSED               = (0),
	SOCKET_LISTEN               = (1),
	SOCKET_SYN_SENT             = (2),
	SOCKET_SYN_RCVD             = (3),
	SOCKET_ESTABLISHED          = (4),
	SOCKET_FIN_WAIT_1           = (5),
	SOCKET_FIN_WAIT_2           = (6),
	SOCKET_CLOSE_WAIT           = (7),
	SOCKET_CLOSING              = (8),
	SOCKET_LAST_ACK             = (9),
	SOCKET_TIME_WAIT            = (10)
} esp32_socket_enum_t;

typedef enum
{
	WL_IDLE_STATUS              = (0),
	WL_NO_SSID_AVAIL            = (1),
	WL_SCAN_COMPLETED           = (2),
	WL_CONNECTED                = (3),
	WL_CONNECT_FAILED           = (4),
	WL_CONNECTION_LOST          = (5),
	WL_DISCONNECTED             = (6),
	WL_AP_LISTENING             = (7),
	WL_AP_CONNECTED             = (8),
	WL_AP_FAILED                = (9),
	WL_NO_MODULE                = (0xFE),
	WL_NO_SHIELD                = (0xFF)
} esp32_wlan_enum_t;

typedef enum
{
	TCP_MODE                    = (0),
	UDP_MODE                    = (1),
	TLS_MODE                    = (2),
	UDP_MODE_2                  = (3)
} esp32_socket_mode_enum_t;

/* clang-format on */

typedef void (*esp32_params_del)(void *arg);

typedef struct
{
	uint32_t param_len;
	uint8_t *param;
} esp32_param_t;

typedef struct
{
	uint32_t params_num;
	esp32_param_t **params;
	esp32_params_del del;
} esp32_params_t;

//typedef void (*esp32_aps_list_del)(void *arg);

//typedef struct
//{
//    int8_t rssi;
//    uint8_t encr;
//    uint8_t ssid[33];
//} esp32_ap_t;

//typedef struct
//{
//    uint32_t aps_num;
//    esp32_ap_t **aps;
//    esp32_aps_list_del del;
//} esp32_aps_list_t;

typedef struct
{
	uint8_t localIp[32];
	uint8_t subnetMask[32];
	uint8_t gatewayIp[32];
} esp32_net_t;

typedef struct ESP32_Init {
	const char                  *ssid;
	const char                  *pass;
	uint8_t                     retry_times;
	uint8_t                     cs_num;
	uint8_t                     rst_num;
	uint8_t                     rdy_num;
	SPI_Handle_t                *hspi;
	I2C_Handle_t                *hi2c;
} ESP32_Init_t;

typedef struct ESP32_Handle {
	ESP32_Init_t                Init;		/* ESP32初期設定パラメータ */
	uint8_t                     socket_num; /* ソケット番号 */
	esp32_wlan_enum_t           connection;
	esp32_socket_enum_t         status;
	unsigned int                timeout;
	SPI_Handle_t                *hspi;
	I2C_Handle_t                *hi2c;
} ESP32_Handle_t;

void esp32_set_debug(ESP32_Handle_t *hesp, uint8_t debug);
ER esp32_firmware_version(ESP32_Handle_t *hesp, char* fw_version);
int8_t esp32_connect(ESP32_Handle_t *hesp);
int8_t esp32_disconnect(ESP32_Handle_t *hesp);
int8_t esp32_status(ESP32_Handle_t *hesp);
uint32_t esp32_get_time(ESP32_Handle_t *hesp);
void esp32_set_certificate(ESP32_Handle_t *hesp, char *client_ca);
void esp32_set_private_key(ESP32_Handle_t *hesp, char *private_key);

int esp32_get_host_by_name(ESP32_Handle_t *hesp, uint8_t *hostname, uint8_t *ip);
uint8_t esp32_get_socket(ESP32_Handle_t *hesp);
esp32_socket_enum_t esp32_socket_status(ESP32_Handle_t *hesp);
int esp32_socket_available(ESP32_Handle_t *hesp);
uint8_t esp32_socket_connected(ESP32_Handle_t *hesp);
int8_t esp32_socket_connect(ESP32_Handle_t *hesp, uint8_t *dest, uint8_t dest_type, uint16_t port, esp32_socket_mode_enum_t conn_mod);
uint32_t esp32_socket_write(ESP32_Handle_t *hesp, uint8_t *buffer, uint16_t len);
int esp32_socket_read(ESP32_Handle_t *hesp, uint8_t *buff, uint16_t size);
int8_t esp32_socket_close(ESP32_Handle_t *hesp);

char *socket_enum_to_str(esp32_socket_enum_t x);
char *wlan_enum_to_str(esp32_wlan_enum_t x);

ESP32_Handle_t * esp32_init(ESP32_Init_t *init);
ER esp32_deinit(ESP32_Handle_t *hesp);
