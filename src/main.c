/*
 * Copyright (c) 2022 Lukasz Majewski, DENX Software Engineering GmbH
 * Copyright (c) 2019 Peter Bigot Consulting, LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Sample which uses the filesystem API with littlefs */


#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

#include <zephyr/types.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>
#include <zephyr/settings/settings.h>
#include <stdio.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

#include <string.h>

#include "pins_handler.h"


enum MenuMode{IDLE,START,GETID,ERASEMEM,PROGRAM,SAVEDATA};
static enum MenuMode menu;


/*LOG NAME*/
#define LOG_MODULE_NAME fota_project
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/*UART1*/
#define RECEIVE_TIMEOUT 	100
#define Nmax 				255
#define RECEIVE_BUFF_SIZE 	10

#define UART_DEVICE_NODE1 DT_CHOSEN(zephyr_shell_uart)
static const struct device *const uart_dev1 = DEVICE_DT_GET(UART_DEVICE_NODE1);

const uint32_t START_ADD = 0x08000000;
static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};
static uint8_t uart1_response[RECEIVE_BUFF_SIZE];
static bool response = false;


/*BLE-UART0 (NUS)  CONFIG*/
#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE 
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

/*BLE-UART0 (NUS)  CONFIG*/
static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
};

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len);

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

/*LITTLEFS CONFIG*/
#define MAX_PATH_LEN 	255
#define SIZE_PACK_BLE 	100
#define READ_SIZE 		7680 // 256 (size of a page) * 30 (number of pages)

static uint8_t data_receive_ble[SIZE_PACK_BLE];
static uint8_t file_read[READ_SIZE];

char fname[MAX_PATH_LEN];
struct fs_file_t file;
uint32_t nbytes = 0;

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storagedutprogram);
static struct fs_mount_t lfs_storage_mnt = {
	.type = FS_LITTLEFS,
	.fs_data = &storagedutprogram,
	.storage_dev = (void *)FIXED_PARTITION_ID(slot1_partition),
	.mnt_point = "/lfs1",
};

struct fs_mount_t *mp = &lfs_storage_mnt;


/*FUNCTIONS LITTLEFS*/
static int lsdir(const char *path)
{
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res) {
		LOG_ERR("Error opening dir %s [%d]\n", path, res);
		return res;
	}

	LOG_PRINTK("\nListing dir %s ...\n", path);
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			if (res < 0) {
				LOG_ERR("Error reading dir [%d]\n", res);
			}
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			LOG_PRINTK("[DIR ] %s\n", entry.name);
		} else {
			LOG_PRINTK("[FILE] %s (size = %zu)\n",
				   entry.name, entry.size);
		}
	}

	/* Verify fs_closedir() */
	fs_closedir(&dirp);

	return res;
}

static void save_data(uint8_t *p, struct uart_data_t *tx)
{
	struct fs_dirent dirent;
	int rc,i;


	/*
	 * Uncomment below line to force re-creation of the test pattern
	 * file on the littlefs FS.
	 */
	//fs_unlink(fname); //force re-recreation
	//fs_file_t_init(&file);

/* 	rc = fs_open(&file, fname, FS_O_RDWR);
	if (rc < 0) {
		LOG_ERR("FAIL: open %s: %d", fname, rc);
		return rc;
	} */
	/*If the first byte is command 0x80 - Reset file*/
	if(tx->data[0] == 0x80){
		fs_unlink(fname);
		fs_file_t_init(&file);
		rc = fs_open(&file, fname, FS_O_CREATE | FS_O_RDWR);
		if (rc < 0) {
			LOG_ERR("FAIL: open %s: %d", fname, rc);
			return rc;
		}	
	}

/* 	rc = fs_stat(fname, &dirent);
	if (rc < 0) {
		LOG_ERR("FAIL: stat %s: %d", fname, rc);
	} */

	for(i = 1;i<61;i++){
		data_receive_ble[i-1]=tx->data[i];
	}

	//printk("------ FILE: %s ------\n", fname);
	//print_file(data_receive_ble, sizeof(data_receive_ble));

	//printk("fs_tell : %d\n",fs_tell(&file));
	//printk("offset  : %d\n",offset);
	rc = fs_seek(&file,0, FS_SEEK_END);
	if (rc < 0) {
		LOG_ERR("FAIL: seek %s: %d", fname, rc);
	}
	
	rc = fs_write(&file, data_receive_ble, (tx->len-1));
	if (rc < 0) {
		LOG_ERR("FAIL: write %s: %d", fname, rc);
	}

	/*flush tab*/
	for(int i = 0;i<sizeof(data_receive_ble);i++){
		data_receive_ble[i]=0;
	}

	nbytes = fs_tell(&file);
	printk("%d \r\n",nbytes);

	return (rc < 0 ? rc : 0); 
}

void print_file(uint8_t *p, uint16_t size)
{
	int i, j = size / 16, k;

	for (k = 0, i = 0; k < j; i += 16, k++) {
		printk("%02x %02x %02x %02x %02x %02x %02x %02x ",
			   p[i], p[i+1], p[i+2], p[i+3],
			   p[i+4], p[i+5], p[i+6], p[i+7]);
		printk("%02x %02x %02x %02x %02x %02x %02x %02x\n",
			   p[i+8], p[i+9], p[i+10], p[i+11],
			   p[i+12], p[i+13], p[i+14], p[i+15]);

		/* Mark 512B (sector) chunks of the test file */
		if ((k + 1) % 32 == 0) {
			printk("\n");
		}
	}

	for (; i < size; i++) {
		printk("%02x ", p[i]);
	}

	printk("\n");
}

static int littlefs_flash_erase(unsigned int id)
{
	const struct flash_area *pfa;
	int rc;

	rc = flash_area_open(id, &pfa);
	if (rc < 0) {
		LOG_ERR("FAIL: unable to find flash area %u: %d\n",
			id, rc);
		return rc;
	}

	LOG_PRINTK("Area %u at 0x%x on %s for %u bytes\n",
		   id, (unsigned int)pfa->fa_off, pfa->fa_dev->name,
		   (unsigned int)pfa->fa_size);

	/* Optional wipe flash contents */
	if (IS_ENABLED(CONFIG_APP_WIPE_STORAGE)) {
		rc = flash_area_erase(pfa, 0, pfa->fa_size);
		LOG_ERR("Erasing flash area ... %d", rc);
	}

	flash_area_close(pfa);
	return rc;
}

static int littlefs_mount(struct fs_mount_t *mp)
{
	int rc;

	rc = littlefs_flash_erase((uintptr_t)mp->storage_dev);
	if (rc < 0) {
		return rc;
	}

	rc = fs_mount(mp);
	if (rc < 0) {
		LOG_PRINTK("FAIL: mount id %" PRIuPTR " at %s: %d\n",
		       (uintptr_t)mp->storage_dev, mp->mnt_point, rc);
		return rc;
	}
	printk("%s mount: %d\n", mp->mnt_point, rc);

	return 0;
}

void littlefs_init(){
	struct fs_statvfs sbuf;
	int rc;

	printk("Sample program to r/w files on littlefs\n");
	rc = littlefs_mount(mp);
	if (rc < 0) {
		return;
	}

	snprintf(fname, sizeof(fname), "%s/data.bin", mp->mnt_point);

	fs_file_t_init(&file);

	rc = fs_open(&file, fname, FS_O_CREATE | FS_O_RDWR);
	if (rc < 0) {
		LOG_ERR("FAIL: open %s: %d", fname, rc);
		return rc;
	}	

	//read mem
	rc = fs_read(&file, file_read, sizeof(file_read));
	if (rc < 0) {
		LOG_ERR("FAIL: read %s: [rd:%d]", fname, rc);
		
	}
	printk("------ FILE MEM: %s ------\n", fname);
	print_file(file_read, sizeof(file_read));

	rc = fs_statvfs(mp->mnt_point, &sbuf);
	if (rc < 0) {
		LOG_PRINTK("FAIL: statvfs: %d\n", rc);

	}

	printk("%s: bsize = %lu ; frsize = %lu ;"
		   " blocks = %lu ; bfree = %lu\n",
		   mp->mnt_point,
		   sbuf.f_bsize, sbuf.f_frsize,
		   sbuf.f_blocks, sbuf.f_bfree);

	rc = lsdir(mp->mnt_point);
	if (rc < 0) {
		LOG_PRINTK("FAIL: lsdir %s: %d\n", mp->mnt_point, rc);

	}

	rc = fs_seek(&file, 0, FS_SEEK_END);
	if (rc < 0) {
		LOG_ERR("FAIL: seek %s: %d", fname, rc);
	}
	nbytes = fs_tell(&file);
	printk("nbytes: %d \r\n",nbytes);

}

/*FUNCTIONS BLE-UART0 (NUS)*/
void nus_init(){
	int ret;

	ret = uart_init();
	if (ret) {
		printk("UART0 device not found!");
		error();
	}

	ret = bt_enable(NULL);
	if (ret) {
		printk("ble error");
		error();
	}
	printk("Bluetooth initialized\n");
	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	ret = bt_nus_init(&nus_cb);
	if (ret) {
		LOG_ERR("Failed to initialize UART service (err: %d)", ret);
		return;
	}

	ret = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (ret) {
		LOG_ERR("Advertising failed to start (err %d)", ret);
		return;
	}
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
					   data);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		LOG_DBG("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		buf->len += evt->data.rx.len;

		if (disable_req) {
			return;
		}

		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
		    (evt->data.rx.buf[buf->len - 1] == '\r')) {
			disable_req = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
			       UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART_RX_BUF_RELEASED");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data);

		if (buf->len > 0) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else {
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf) {
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
				   data);

		uart_tx(uart, &buf->data[aborted_len],
			buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if (!device_is_ready(uart)) {
		return -ENODEV;
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		k_free(rx);
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}


	tx = k_malloc(sizeof(*tx));

	if (tx) {
		pos = snprintf(tx->data, sizeof(tx->data),
			       "Starting Nordic UART service example\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data))) {
			k_free(rx);
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	} else {
		k_free(rx);
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		k_free(rx);
		k_free(tx);
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), 50);
	if (err) {
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
		/* Free the rx buffer only because the tx buffer will be handled in the callback */
		k_free(rx);
	}

	return err;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int rc;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	printk("------ FLASH : START ------\n");
	start();
	printk("------ FLASH : ERASE ------\n");
	printk("fs_tell : %d\n",fs_tell(&file));
	erase_flash_dut();
	printk("------ FLASH : PROGRAM ------\n");
	program_dut();
	boot_activation(false);
	reset_stm();
}

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};
	int rc;

	//bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));
	//LOG_INF("Received data from: %s", addr);
	
	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx) {
			LOG_WRN("Not able to allocate UART send data buffer");
			return;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size) {
			tx->len = tx_data_size;
		} else {
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		//uart print
		/* err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		printk(" \n"); */
		/* if (err) {
			k_fifo_put(&fifo_uart_tx_data, tx);
		} */

		//write on flash
		save_data(fname,tx);

		k_free(tx);

	}
	
}


void error(void)
{
	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

void close_and_unmount(){
	int ret,rc;
	ret = fs_close(&file);
	printk("------ FILE CLOSED ------\n");
	if (ret < 0) {
		LOG_ERR("FAIL: close %s: %d", fname, ret);
		return ret;
	}

	rc = fs_unmount(mp);
	printk("%s unmount: %d\n", mp->mnt_point, rc);
}

/*FUNCTIONS UART1*/
static void uart1_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{

	switch (evt->type) {
	case UART_RX_RDY:
		//printk("UART_RX_RDY\n");
		//printk("Response received : ");
		for (int i = 0; i<evt->data.rx.len;i++){
			uart1_response[i] = evt->data.rx.buf[evt->data.rx.offset];
			//printk("%x",uart1_response[i]);
		}
		//printk("\n");
		response = true;

		break;

	case UART_RX_DISABLED:
		printk("UART_RX_DISABLED\n");
		uart_rx_enable(uart_dev1, rx_buf, sizeof(rx_buf),RECEIVE_TIMEOUT);
		break;

	default:
		break;
	}
}

void print_uart1(uint8_t buf)
{
	uart_tx(uart_dev1, &buf,sizeof(buf),SYS_FOREVER_MS);
	k_msleep(1);
}

void uart1_init(){
	int ret;
	/*UART1 INIT*/
	if (!device_is_ready(uart_dev1)) {
		printk("UART device not found!");
		return;
	}
	ret = uart_callback_set(uart_dev1, uart1_cb, NULL);
	if (ret) {
		printk("Cannot initialize UART callback");
	}

	uart_rx_enable(uart_dev1, rx_buf, sizeof(rx_buf), RECEIVE_TIMEOUT);
	printk("uart enabled\n");
}


/*FUNCTIONS STMBOOTLOADER*/
bool wait_ack(char message[])
{
	bool ret = false;
	//message for debug
	//print_uart(message);
	while (response == false){};

	//printk("Response: ");
	if(uart1_response[0] == 'y' ){
		response = false; //reset flag
		//printk("ACK\r\n");
		//printk("\r\n");
		//printk(message);
		ret = true;
	}else if (uart1_response[0] == 0x1f) {
		printk("NACK \r\n");
		ret = false;}
	else{
		printk("Unknown response \r\n");
	}
	
	return ret;
}

void start(){
	print_uart1(0x7f); //start message
	//wait uc response
	wait_ack("***START ACK***\r\n");

}

void erase_flash_dut(){

	uint8_t data[2];
	uint16_t NpagesToErase;
	uint8_t checksum;

	data[0] = 0x44;
	data[1] = 0xBB;
	print_uart1(data[0]);
	print_uart1(data[1]); 
	if(wait_ack("***ERASE MEM ACK***\r\n")){
		//number of pages to erase on two bytes
		NpagesToErase = 700; //256kB = 256(bytes in one page)*1000(number of pages)
		data[0] = (NpagesToErase & 0xff00)>>8; 
		data[1] = NpagesToErase & 0x00ff;
		print_uart1(data[0]);
		print_uart1(data[1]);
		checksum = data[0]^data[1];

		//pages to erase
		for(int i = 0; i<(NpagesToErase+1);i++){
			data[0] = (i & 0xff00) >> 8; 
			data[1] = (i & 0x00ff);

			checksum = checksum^data[0];
			checksum = checksum^data[1];
			print_uart1(data[0]);
			print_uart1(data[1]);

			printk("PAGE : %x %x \r\n",data[0],data[1]);

			//wait_ack("***PAGE NUMBER ACK***\r\n");
		}
		print_uart1(checksum);

		if(wait_ack("***GLOBAL ERASE ACK***\r\n")){
			printk("Full memory erased \r\n");
		}

	}
	//some delay to erase the flash 
	k_msleep(1000);		
	
}

void program_dut(){
	uint8_t data[2];
	uint32_t add_count;
	uint8_t checksum_mem = 0;
	uint8_t add[5];
	uint8_t pages_in_onefile = (sizeof(file_read)/256);
	uint16_t total_pages;
	uint8_t read_file_loop;
	int rc;
	uint8_t data_page[Nmax+1];
	
	//printk("Total pages :%d\r\n",pages);

	rc = fs_seek(&file, 0, FS_SEEK_END);
	if (rc < 0) {
		LOG_ERR("FAIL: seek %s: %d", fname, rc);
		//goto out;
	}

	nbytes =fs_tell(&file);
	printk("nbytes: %d\n",nbytes);

	total_pages = (nbytes/256)+1;
	printk("nbytes: %d\n",nbytes);

	read_file_loop = (total_pages/pages_in_onefile)+1;
	
	data[0] = 0x31;
	data[1] = 0xCE;
	print_uart1(data[0]);
	k_msleep(1);
	print_uart1(data[1]);	


	add_count = START_ADD;
	//checksum data to send -> constant for testing

	if(wait_ack("***WRITE MEM ACK***\r\n")){
		for(int l = 0;l<read_file_loop;l++){

			rc = fs_seek(&file, 0, FS_SEEK_SET+(l*sizeof(file_read)));
			if (rc < 0) {
				LOG_ERR("FAIL: seek %s: %d", fname, rc);
				//goto out;
			}

			rc = fs_read(&file, file_read, sizeof(file_read));
			if (rc < 0) {
				LOG_ERR("FAIL: read %s: [rd:%d]", fname, rc);
				//goto out;
			} 

			for(int j = 0; j<pages_in_onefile;j++){
				checksum_mem = 0;
				checksum_mem = checksum_mem^0;
				checksum_mem = checksum_mem ^ Nmax;
				
				add[0] = add_count>>24;
				add[1] = (add_count>>16)&0x00ff;
				add[2] = (add_count>>8)&0x00ff;
				add[3] = add_count&0x00ff;
				add[4] = add[0] ^ add[1] ^ add[2] ^ add[3];

				if(j+(l*30) > total_pages){
					break;
				}
				
				print_uart1(add[0]);
				print_uart1(add[1]);
				print_uart1(add[2]);
				print_uart1(add[3]);				
				print_uart1(add[4]); 
				
				if(wait_ack("***ADD + CHECHSUM ACK***\r\n")){
					data[0] = Nmax;
					print_uart1(data[0]);

					//fill new page to push in the flash of the stm
					for(int k = 0;k<=Nmax;k++){

						data_page[k] = file_read[k+(j*(Nmax+1))];
						checksum_mem = checksum_mem ^ data_page[k];
					}
					data[1] = checksum_mem;

					//printk("index page : ");
					//push the page in the flash of the stm
					for(int i = 0;i<=Nmax ; i++){
						print_uart1(data_page[i]);
						//k_msleep(1);
					}
					//printk("\r\n");
					//k_msleep(3);
					
					//uart_tx(uart_dev1,data_page,sizeof(data_page),SYS_FOREVER_US);
					
					//checksum
					print_uart1(data[1]);
					k_msleep(1);
					//printk("Size page to write :%d\r\n",sizeof(data_page));
					//printk("checksum : %d\r\n",data[1]);

					if(wait_ack("***N BYTES + DATA + CHECKSUM ACK***\r\n"))
					{
						add_count += 256;
						data[0] = 0x31;
						data[1] = 0xCE;
						print_uart1(data[0]);
						print_uart1(data[1]);
						wait_ack("***NEXT PAGE ACK***\r\n");
					}
				}
				printk("------ PAGE %d PROGRAMMED %d/%d ------ NEXT PAGE ADD : %x ------\n",(j+(l*30)),(l+1),read_file_loop,add_count);			
			}
		}
	}
	close_and_unmount();
}


void main(void)
{
	pins_init();	
	littlefs_init(); // LITTLEFS INIT
	nus_init(); //UART0-BLE 
	uart1_init(); //UART1

	menu = IDLE;

/* 	printk("------ FILE READ: ------\n");
	print_file(file_read, sizeof(file_read)); */
 	
/*  	k_msleep(3000);
	printk("------ FLASH : START ------\n");
	start();
	printk("------ FLASH : ERASE ------\n");
	erase_flash_dut();
	printk("------ FLASH : PROGRAM ------\n");
	program_dut();  */

	while(true){
		switch (menu)
		{
		case IDLE:
			/* code */
			break;
		case START:
			/* code */
			break;
		case GETID:
			/* code */
			break;						
		case ERASEMEM:
			/* code */
			break;
		case PROGRAM:
			/* code */
			break;						
		case SAVEDATA:
			/* code */
			break;			
		default:
			break;
		}
	}

}


void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);

	for (;;) {
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		if (bt_nus_send(NULL, buf->data, buf->len)) {
			LOG_WRN("Failed to send data over BLE connection");
		}

		k_free(buf);
	}
}
K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);
