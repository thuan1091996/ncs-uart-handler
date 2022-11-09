#include "uart_handler.h"
#include <sys/ring_buffer.h>
#include <string.h>

K_SEM_DEFINE(tx_done, 1, 1);

// UART TX fifo
RING_BUF_DECLARE(app_tx_fifo, UART_TX_BUF_SIZE);
volatile int bytes_claimed;

// UART RX primary buffers
#define UART_RX_SLAB_SIZE (UART_RX_BUF_SIZE / 4)
K_MEM_SLAB_DEFINE(memslab_uart_rx, UART_RX_SLAB_SIZE, 4, 4);

// UART RX message queue
K_MSGQ_DEFINE(uart_rx_msgq, sizeof(struct uart_msg_queue_item), UART_RX_MSG_QUEUE_SIZE, 4);
volatile int allocated_slabs = 0;
static bool uart_event_queue_overflow = false;

// UART RX free bytes
static bool rx_enable_request = false;
static bool rx_received_since_enable = false;

static app_uart_event_handler_t app_uart_event_handler;

static const struct device *dev_uart;

static int uart_tx_get_from_queue(void)
{
	uint8_t *data_ptr;
	// Try to claim any available bytes in the FIFO
	bytes_claimed = ring_buf_get_claim(&app_tx_fifo, &data_ptr, UART_TX_BUF_SIZE);

	if(bytes_claimed > 0) {
		// Start a UART transmission based on the number of available bytes
		uart_tx(dev_uart, data_ptr, bytes_claimed, SYS_FOREVER_MS);
	}
	return bytes_claimed;
}

static inline void uart_push_event(struct uart_msg_queue_item *event)
{
	if(k_msgq_put(&uart_rx_msgq, event, K_NO_WAIT) != 0){
		printk("Error: Uart event queue full!\n");
		uart_event_queue_overflow = true;
	}
}

void app_uart_async_callback(const struct device *uart_dev,
							 struct uart_event *evt, void *user_data)
{
	static struct uart_msg_queue_item new_message;
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
			//printk("rdy (%i)\n", evt->data.rx.len);
			rx_received_since_enable = true;
			new_message.type = APP_UART_EVT_RX;
			new_message.data.rx.bytes = evt->data.rx.buf + evt->data.rx.offset;
			new_message.data.rx.length = evt->data.rx.len;
			new_message.data.rx._source_buf = evt->data.rx.buf;
			uart_push_event(&new_message);
			break;
		
		case UART_RX_BUF_REQUEST:
			if(k_mem_slab_alloc(&memslab_uart_rx, (void**)&new_rx_buf, K_NO_WAIT) == 0) {
				allocated_slabs++;
				//printk("Alloc (%i)\n", allocated_slabs);
				uart_rx_buf_rsp(dev_uart, new_rx_buf, UART_RX_SLAB_SIZE);
			}
			break;

		case UART_RX_BUF_RELEASED:
			break;

		case UART_RX_STOPPED:
			new_message.type = APP_UART_EVT_ERROR;
			new_message.data.error.reason = evt->data.rx_stop.reason;
			uart_push_event(&new_message);
			break;

		case UART_RX_DISABLED:
			//printk("dis\n");
			if((ret = k_mem_slab_alloc(&memslab_uart_rx, (void**)&new_rx_buf, K_NO_WAIT)) == 0) {
				//uart_rx_buf_rsp(dev_uart, new_rx_buf, UART_RX_SLAB_SIZE);
				allocated_slabs++;
				//printk("RX DIS. Re-enabling (%i)\n", allocated_slabs);
				int ret = uart_rx_enable(dev_uart, new_rx_buf, UART_RX_SLAB_SIZE, UART_RX_TIMEOUT_US);
				if(ret) {
					printk("UART rx enable error in disable callback: %d\n", ret);
					return;
				}
			} else {
				//printk("RX dis, buf full. Set flag %i\n", ret);
				rx_enable_request = true;
			}
			rx_received_since_enable = false;
			break;
		
		default:
			break;
	}
}

void app_uart_init(app_uart_event_handler_t evt_handler)
{
	int ret;
	dev_uart = DEVICE_DT_GET(DT_NODELABEL(my_uart));
	if(!device_is_ready(dev_uart)) {
		printk("UART device not ready!\n");
		return;
	}

	app_uart_event_handler = evt_handler;

	ret = uart_callback_set(dev_uart, app_uart_async_callback, NULL);
	if(ret) {
		printk("Uart callback set error: %d\n", ret);
		return;
	}

	char *rx_buf_ptr;
	if (k_mem_slab_alloc(&memslab_uart_rx, (void**)&rx_buf_ptr, K_NO_WAIT) == 0) {
		allocated_slabs++;
		ret = uart_rx_enable(dev_uart, rx_buf_ptr, UART_RX_SLAB_SIZE, UART_RX_TIMEOUT_US);
		if(ret) {
			printk("UART rx enable error: %d\n", ret);
			return;
		}
	}
}

// Function to send UART data, by writing it to a ring buffer (FIFO) in the application
// WARNING: This function is not thread safe! If you want to call this function from multiple threads a semaphore should be used
int app_uart_send(const uint8_t * data_ptr, uint32_t data_len, k_timeout_t timeout)
{
	while(1) {
		// Try to move the data into the TX ring buffer
		uint32_t written_to_buf = ring_buf_put(&app_tx_fifo, data_ptr, data_len);
		data_len -= written_to_buf;
		
		// In case the UART TX is idle, start transmission
		if(k_sem_take(&tx_done, K_NO_WAIT) == 0) {
			uart_tx_get_from_queue();
		}	
		
		// In case all the data was written, exit the loop
		if(data_len == 0) break;

		// If timeout if 0, exit immediately
		if(timeout.ticks == 0) break;

		// In case some data is still to be written, sleep for some time and run the loop one more time
		k_msleep(10);
		data_ptr += written_to_buf;
	}

	return 0;
}

char *last_read_buffer = 0;
int app_uart_rx(uint8_t ** data_ptr, uint32_t * data_len, k_timeout_t timeout)
{
	int ret;
	static struct uart_msg_queue_item incoming_message;

	// Check for a new message in the buffer
	ret = k_msgq_get(&uart_rx_msgq, &incoming_message, timeout);
	if(ret != 0) return ret;

	*data_ptr = incoming_message.data.rx.bytes;
	*data_len = incoming_message.data.rx.length;
	last_read_buffer = incoming_message.data.rx._source_buf;

	return 0;
}

char *last_freed_buffer = 0;
int app_uart_rx_free(void)
{
	int ret;

	// Remove the last message in the buffer
	if(last_freed_buffer != 0 && last_freed_buffer != last_read_buffer){
		k_mem_slab_free(&memslab_uart_rx, (void**)&last_freed_buffer);
		allocated_slabs--;
		//printk("dealloc (%i)\n", allocated_slabs);

		if(rx_enable_request) {
			rx_enable_request = false;
			//printk("Buffers freed. Re-en RX\n");
			char *rx_buf_ptr;
			if (k_mem_slab_alloc(&memslab_uart_rx, (void**)&rx_buf_ptr, K_NO_WAIT) == 0) {
				ret = uart_rx_enable(dev_uart, rx_buf_ptr, UART_RX_SLAB_SIZE, UART_RX_TIMEOUT_US);
				if(ret) {
					printk("UART rx enable error: %d\n", ret);
					return -1;
				}
				allocated_slabs++;
			} else printk("Memslab alloc failed!!!\n");
		}
	}
	last_freed_buffer = last_read_buffer;

	return 0;
}

void uart_event_thread_func(void)
{
	static struct uart_msg_queue_item new_event;
	while(1) {
		// Wait for a new event to be available in the event queue
		k_msgq_get(&uart_rx_msgq, &new_event, K_FOREVER);

		// In case of an RX event we need to handle freeing of the RX buffers (memslab_uart_rx)
		if(new_event.type == APP_UART_EVT_RX) {
			last_read_buffer = new_event.data.rx._source_buf;
			app_uart_event_handler(&new_event);
			app_uart_rx_free();
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

K_THREAD_DEFINE(app_uart_evt_thread, UART_EVENT_THREAD_STACK_SIZE, uart_event_thread_func, 0, 0, 0, UART_EVENT_THREAD_PRIORITY, 0, 100);