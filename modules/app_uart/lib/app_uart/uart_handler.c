#include <uart_handler.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>

#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME app_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define UART_INSTANCE CONCAT(uart, CONFIG_APP_UART_HW_INDEX)

K_SEM_DEFINE(tx_done, 1, 1);

// UART TX fifo
RING_BUF_DECLARE(app_tx_fifo, CONFIG_APP_UART_TX_BUF_SIZE);
volatile int bytes_claimed = 0;

// UART RX primary buffers
#define UART_RX_SLAB_SIZE (CONFIG_APP_UART_RX_BUF_SIZE / 4)
K_MEM_SLAB_DEFINE(memslab_uart_rx, UART_RX_SLAB_SIZE, 4, 4);

// UART RX message queue
K_MSGQ_DEFINE(uart_evt_queue, sizeof(struct app_uart_evt_t), CONFIG_APP_UART_EVT_QUEUE_SIZE, 4);
volatile int allocated_slabs = 0;
volatile int rx_bytes_in_buffers = 0;
static bool uart_event_queue_overflow = false;

// UART RX free bytes
static bool rx_enable_request = false;
static bool rx_received_since_enable = false;

static uint32_t error_flags = 0;

static app_uart_event_handler_t app_uart_event_handler;

static const struct device *dev_uart;

static int uart_tx_get_from_queue(void)
{
	uint8_t *data_ptr;
	// Try to claim any available bytes in the FIFO
	bytes_claimed = ring_buf_get_claim(&app_tx_fifo, &data_ptr, CONFIG_APP_UART_TX_BUF_SIZE);

	if(bytes_claimed > 0) {
		// Start a UART transmission based on the number of available bytes
		uart_tx(dev_uart, data_ptr, bytes_claimed, SYS_FOREVER_MS);
	}
	return bytes_claimed;
}

static inline void uart_push_event(struct app_uart_evt_t *event)
{
	if(k_msgq_put(&uart_evt_queue, event, K_NO_WAIT) != 0){
		LOG_ERR("Error: Uart event queue full!");
		uart_event_queue_overflow = true;
	}
}

void app_uart_async_callback(const struct device *uart_dev,
							 struct uart_event *evt, void *user_data)
{
	static struct app_uart_evt_t new_message;
	static char *new_rx_buf;
	int ret;

	switch (evt->type) {
		case UART_TX_DONE:
			// Free up the written bytes in the TX FIFO
			ring_buf_get_finish(&app_tx_fifo, bytes_claimed);

			// If there is more data in the TX fifo, start the transmission
			if(uart_tx_get_from_queue() == 0) {
				// Or release the semaphore if the TX fifo is empty
				k_sem_give(&tx_done);
			}
			break;
		
		case UART_RX_RDY:
			LOG_DBG("rdy (%i)", evt->data.rx.len);
			rx_received_since_enable = true;
			new_message.type = APP_UART_EVT_RX;
			new_message.data.rx.bytes = evt->data.rx.buf + evt->data.rx.offset;
			new_message.data.rx.length = evt->data.rx.len;
			new_message.data.rx._source_buf = evt->data.rx.buf;
			uart_push_event(&new_message);
			rx_bytes_in_buffers += evt->data.rx.len;
		//Dump the receive buffer
		LOG_HEXDUMP_DBG(evt->data.rx.buf + evt->data.rx.offset, evt->data.rx.len, "RX");
		if (evt->data.rx.buf[evt->data.rx.offset + evt->data.rx.len -1] == '\n')
		{
			LOG_INF("End line detected, size: %d", new_message.data.rx.length);
			uart_rx_disable(dev_uart);
		}
			break;
		
		case UART_RX_BUF_REQUEST:
			if(k_mem_slab_alloc(&memslab_uart_rx, (void**)&new_rx_buf, K_NO_WAIT) == 0) {
				allocated_slabs++;
				LOG_DBG("Alloc (%i)", allocated_slabs);
				uart_rx_buf_rsp(dev_uart, new_rx_buf, UART_RX_SLAB_SIZE);
			}
			break;

		case UART_RX_BUF_RELEASED:
			break;

		case UART_RX_STOPPED:
			new_message.type = APP_UART_EVT_ERROR;
			if(error_flags == 0) {
				error_flags = evt->data.rx_stop.reason;
				//new_message.data.error.reason = evt->data.rx_stop.reason;
				uart_push_event(&new_message);			
			} else {
				error_flags |= evt->data.rx_stop.reason;
			}
			break;

		case UART_RX_DISABLED:
			if(rx_bytes_in_buffers == 0) {
				k_mem_slab_init(&memslab_uart_rx, _k_mem_slab_buf_memslab_uart_rx, UART_RX_SLAB_SIZE, 4);
				allocated_slabs = 0;
			}	
			if((ret = k_mem_slab_alloc(&memslab_uart_rx, (void**)&new_rx_buf, K_NO_WAIT)) == 0) {
				allocated_slabs++;
				LOG_DBG("RX DIS. Re-enabling (%i)", allocated_slabs);
				int ret = uart_rx_enable(dev_uart, new_rx_buf, UART_RX_SLAB_SIZE, CONFIG_APP_UART_RX_TIMEOUT_US);
				if(ret) {
					LOG_ERR("UART rx enable error in disable callback: %d\n", ret);
					return;
				}
			} else {
				LOG_DBG("RX dis, buf full. Set flag %i", ret);
				rx_enable_request = true;
			}
			rx_received_since_enable = false;
			break;
		
		default:
			break;
	}
}

int app_uart_init(app_uart_event_handler_t evt_handler)
{
	int ret;

	dev_uart = DEVICE_DT_GET(DT_CHOSEN(nordic_app_uart));
	if(!device_is_ready(dev_uart)) {
		LOG_ERR("UART device not ready!");
		return -ENODEV;
	}

	app_uart_event_handler = evt_handler;

	ret = uart_callback_set(dev_uart, app_uart_async_callback, NULL);
	if(ret) {
		LOG_ERR("Uart callback set error: %d", ret);
		return ret;
	}

	char *rx_buf_ptr;
	if (k_mem_slab_alloc(&memslab_uart_rx, (void**)&rx_buf_ptr, K_NO_WAIT) == 0) {
		allocated_slabs++;
		ret = uart_rx_enable(dev_uart, rx_buf_ptr, UART_RX_SLAB_SIZE, CONFIG_APP_UART_RX_TIMEOUT_US);
		if(ret) {
			LOG_ERR("UART rx enable error: %d", ret);
			return ret;
		}
	}
	LOG_INF("INITIALIZED");
	return 0;
}

// Function to send UART data, by writing it to a ring buffer (FIFO) in the application
// WARNING: This function is not thread safe! If you want to call this function from multiple threads a semaphore should be used
int app_uart_send(const uint8_t * data_ptr, uint32_t data_len, k_timeout_t timeout)
{
	uint32_t written_to_buf;

	// In case the UART TX is idle, start transmission immediately without buffering
	if(k_sem_take(&tx_done, K_NO_WAIT) == 0) {
		// Start a UART transmission
		uart_tx(dev_uart, data_ptr, data_len, SYS_FOREVER_MS);
		return 0;
	} else {
		// In case the UART is currently busy, try to move the data into the TX ring buffer instead
		written_to_buf = ring_buf_put(&app_tx_fifo, data_ptr, data_len);
		data_len -= written_to_buf;

		// In case all the data was written, exit the function
		if(data_len == 0) return 0;
	}
	
	data_ptr += written_to_buf;

	// If we reach here it means some bytes still haven't been written to the UART. 
	// Wait for the tx_done semaphore using the provided timeout.
	if(k_sem_take(&tx_done, timeout) == 0) {
		// Start a UART transmission
		uart_tx(dev_uart, data_ptr, data_len, SYS_FOREVER_MS);
		return 0;
	}

	// Time out
	return -EAGAIN;
}

char *last_read_buffer = 0;
char *last_freed_buffer = 0;
static int app_uart_rx_free(void)
{
	int ret;

	// Remove the last message in the buffer
	if(last_freed_buffer != 0 && last_freed_buffer != last_read_buffer){
		k_mem_slab_free(&memslab_uart_rx, (void**)&last_freed_buffer);
		allocated_slabs--;
		//printk("dealloc (%i)\n", allocated_slabs);

		if(rx_enable_request) {
			rx_enable_request = false;
			LOG_DBG("Buffers freed. Re-en RX");
			char *rx_buf_ptr;
			if (k_mem_slab_alloc(&memslab_uart_rx, (void**)&rx_buf_ptr, K_NO_WAIT) == 0) {
				ret = uart_rx_enable(dev_uart, rx_buf_ptr, UART_RX_SLAB_SIZE, CONFIG_APP_UART_RX_TIMEOUT_US);
				if(ret) {
					LOG_ERR("UART rx enable error: %d", ret);
					return -1;
				}
				allocated_slabs++;
			} else LOG_ERR("Memslab alloc failed!");
		}
	}
	last_freed_buffer = last_read_buffer;

	return 0;
}

void uart_event_thread_func(void)
{
	static struct app_uart_evt_t new_event;
	while(1) {
		// Wait for a new event to be available in the event queue
		k_msgq_get(&uart_evt_queue, &new_event, K_FOREVER);

		// In case of an RX event we need to handle freeing of the RX buffers (memslab_uart_rx)
		if(new_event.type == APP_UART_EVT_RX) {
			last_read_buffer = new_event.data.rx._source_buf;
			app_uart_event_handler(&new_event);
			rx_bytes_in_buffers -= new_event.data.rx.length;
			app_uart_rx_free();
		} else if(new_event.type == APP_UART_EVT_ERROR) {
			uint32_t evt_error_flags = error_flags;
			error_flags = 0;
			new_event.data.error.reason = evt_error_flags;
			app_uart_event_handler(&new_event);
		// All other events are simply forwarded to the event handler
		} else {
			app_uart_event_handler(&new_event);
		}
		
		// Check if the event queue was overflowed during the processing of the previous event
		if(uart_event_queue_overflow) {
			uart_event_queue_overflow = false;
			new_event.type = APP_UART_EVT_QUEUE_OVERFLOW;
			app_uart_event_handler(&new_event);
		}
	}
}

K_THREAD_DEFINE(app_uart_evt_thread, CONFIG_APP_UART_EVT_THREAD_STACK_SIZE, uart_event_thread_func, \
				0, 0, 0, CONFIG_APP_UART_EVT_THREAD_PRIORITY, 0, 100);
