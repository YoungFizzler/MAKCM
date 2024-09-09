#include "EspUsbHost.h"
#include "esp_task_wdt.h"
#include <driver/timer.h>
#include "freertos/queue.h"
#include "esp_log.h"
#include <stdarg.h>
#include <stdio.h>
#include <sstream>
#include <iomanip>
#include "freertos/semphr.h"

#define USB_TASK_PRIORITY 1
#define CLIENT_TASK_PRIORITY 2
#define USB_FEATURE_SELECTOR_REMOTE_WAKEUP 1
bool EspUsbHost::deviceConnected = false;
bool EspUsbHost::deviceMouseReady = false;
EspUsbHost::HIDReportDescriptor EspUsbHost::HIDReportDesc;
int EspUsbHost::current_log_level = LOG_LEVEL_OFF;
bool enableYield = false;
const unsigned long ledFlashTime = 25; // Set The LED Flash timer in ms
static SemaphoreHandle_t ledSemaphore;
QueueHandle_t logQueue;
const int MaxLogQueSize = 512;
void flashLED();

void EspUsbHost::begin(void)
{
    usbTransferSize = 0;
    deviceSuspended = false;
    last_activity_time = millis();

    const usb_host_config_t host_config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };


    usb_host_install(&host_config);

    logQueue = xQueueCreate(LOG_QUEUE_SIZE, MaxLogQueSize);


    txQueue = xQueueCreate(LOG_QUEUE_SIZE, LOG_MESSAGE_SIZE);


    xTaskCreate(processLogQueue, "Log Processing", 4096, this, 4, &logTaskHandle);

    xTaskCreate([](void *arg)
                { static_cast<EspUsbHost *>(arg)->tx_esp(arg); },
                "tx_esp", 3400, this, 4, &espTxTaskHandle);

    xTaskCreate([](void *arg)
                { static_cast<EspUsbHost *>(arg)->rx_esp_serial0(arg); },
                "RxTaskSerial0", 3400, this, 5, NULL);

    xTaskCreate(rx_esp, "RxTask", 2900, this, 5, NULL);

    xTaskCreate(usb_lib_task, "usbLibTask", 4096, this, USB_TASK_PRIORITY, NULL);

    xTaskCreate(usb_client_task, "usbClientTask", 4096, this, CLIENT_TASK_PRIORITY, NULL);

    xTaskCreate(monitor_inactivity_task, "MonitorInactivityTask", 3200, this, 3, NULL);

    ledSemaphore = xSemaphoreCreateBinary();

    xTaskCreate(ledFlashTask, "LED Flash Task", 1500, NULL, 1, NULL);
}


void EspUsbHost::rx_esp_serial0(void *command)
{
    EspUsbHost *instance = static_cast<EspUsbHost *>(command);
    while (true)
    {
        if (Serial0.available())
        {
            String incomingCommand = Serial0.readStringUntil('\n');
            incomingCommand.trim();
            if (incomingCommand.length() > 0)
            {
                instance->handleIncomingCommands(incomingCommand);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void EspUsbHost::tx_esp(void *arg)
{
    EspUsbHost *instance = static_cast<EspUsbHost *>(arg);
    char logMsg[LOG_MESSAGE_SIZE];
    uint32_t notificationValue;

    while (true)
    {
        xTaskNotifyWait(0, ULONG_MAX, &notificationValue, portMAX_DELAY);

        while (xQueueReceive(instance->txQueue, &logMsg, portMAX_DELAY) == pdPASS)
        {
            Serial1.print(logMsg);
        }
    }
}

void EspUsbHost::rx_esp(void *command)
{
    EspUsbHost *instance = static_cast<EspUsbHost *>(command);
    while (true)
    {
        if (Serial1.available())
        {
            String incomingCommand = Serial1.readStringUntil('\n');
            incomingCommand.trim();
            if (incomingCommand.length() > 0)
            {
                instance->handleIncomingCommands(incomingCommand);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

bool EspUsbHost::tx_Que(const char *format, ...)
{
    char logMsg[LOG_MESSAGE_SIZE];
    va_list args;
    va_start(args, format);
    int logLength = vsnprintf(logMsg, sizeof(logMsg), format, args);
    va_end(args);

    if (logLength >= sizeof(logMsg))
    {
        Serial1.println("Warning: Log message truncated.");
        logLength = sizeof(logMsg) - 1;
    }

    logMsg[logLength] = '\0';

    if (xQueueSend(txQueue, logMsg, 0) == pdPASS)
    {
        xTaskNotifyGive(espTxTaskHandle);
        return true;
    }
    else
    {
        Serial1.println("Queue Full");
        return false;
    }
}

void EspUsbHost::logRawBytes(const uint8_t *data, size_t length, const std::string &label)
{
    if (LOG_LEVEL_PARSED > EspUsbHost::current_log_level)
    {
        return;
    }

    log(LOG_LEVEL_PARSED, "==============================\n");
    log(LOG_LEVEL_PARSED, "**%s**\n", label.c_str());

    if (length > 10)
    {
        size_t bytesPerRow = length / 4;
        size_t remainingBytes = length % 4;

        size_t i = 0;

        for (int row = 0; row < 4; ++row)
        {
            std::ostringstream rowStream;
            for (size_t j = 0; j < bytesPerRow; ++j, ++i)
            {
                rowStream << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                          << static_cast<int>(data[i]) << " ";
            }
            rowStream << "\n";
            log(LOG_LEVEL_PARSED, "%s", rowStream.str().c_str());
        }

        if (remainingBytes > 0)
        {
            std::ostringstream lastRowStream;
            for (size_t j = 0; j < remainingBytes; ++j, ++i)
            {
                lastRowStream << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                              << static_cast<int>(data[i]) << " ";
            }
            lastRowStream << "\n";
            log(LOG_LEVEL_PARSED, "%s", lastRowStream.str().c_str());
        }
    }
    else
    {
        // New logic: Single row output for length <= 10
        std::ostringstream rowStream;
        for (size_t i = 0; i < length; ++i)
        {
            rowStream << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                      << static_cast<int>(data[i]) << " ";
        }
        rowStream << "\n";
        log(LOG_LEVEL_PARSED, "%s", rowStream.str().c_str());
    }
}



void EspUsbHost::log(int level, const char *format, ...)
{
    if (level != LOG_LEVEL_PARSED || EspUsbHost::current_log_level == LOG_LEVEL_OFF)
    {
        return;
    }

    char logMsg[MaxLogQueSize];
    va_list args;
    va_start(args, format);
    int logLength = vsnprintf(logMsg, sizeof(logMsg), format, args);
    va_end(args);


    if (xQueueSend(logQueue, logMsg, 0) != pdPASS)
    {
        Serial1.println("Error: Failed to send log message to queue.");
        return;
    }
    xTaskNotifyGive(logTaskHandle);
}

void processLogQueue(void* parameter)
{
    EspUsbHost* instance = static_cast<EspUsbHost*>(parameter);
    char logMsg[MaxLogQueSize];

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (xQueueReceive(logQueue, logMsg, 0) == pdPASS)
        {
            if (!instance->tx_Que(logMsg))  
            {
                Serial1.println("Error: Failed to send log message via tx_Que.");
            }
        }
    }
}


static void usb_lib_task(void *arg)
{
    EspUsbHost *instance = static_cast<EspUsbHost *>(arg);

    while (true)
    {
        uint32_t event_flags;
        esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        if (err != ESP_OK)
        {
            instance->log(LOG_LEVEL_ERROR, "usb_host_lib_handle_events() err=%x", err);
            continue;
        }

        if (instance->clientHandle == NULL || !instance->isClientRegistering)
        {
            instance->log(LOG_LEVEL_INFO, "Inform: Registering client...");
            const usb_host_client_config_t client_config = {
                .max_num_event_msg = 10,
                .async = {
                    .client_event_callback = instance->_clientEventCallback,
                    .callback_arg = instance,
                }};

            err = usb_host_client_register(&client_config, &instance->clientHandle);
            instance->log(LOG_LEVEL_INFO, "Inform: usb_host_client_register() status: %d", err);
            if (err != ESP_OK)
            {
                instance->log(LOG_LEVEL_WARN, "Warn: Failed to re-register client, retrying...");
                vTaskDelay(100);
            }
            else
            {
                instance->log(LOG_LEVEL_INFO, "Inform: Client registered successfully.");
                instance->isClientRegistering = true;
            }
        }
    }
}

static void usb_client_task(void *arg)
{
    EspUsbHost *instance = static_cast<EspUsbHost *>(arg);

    while (true)
    {
        if (!instance->isClientRegistering)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        esp_err_t err = usb_host_client_handle_events(instance->clientHandle, portMAX_DELAY);
    }
}

void EspUsbHost::monitor_inactivity_task(void *arg)
{
    EspUsbHost *usbHost = static_cast<EspUsbHost *>(arg);
    const char *TAG = "monitor_inactivity_task";

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(100));

        if (EspUsbHost::deviceConnected && !usbHost->deviceSuspended && millis() - usbHost->last_activity_time > 10000)
        {
            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                usbHost->log(LOG_LEVEL_INFO, "%s: Inform: No activity for 10 seconds, suspending device", TAG);
            }
            usbHost->suspend_device();
        }
    }
}

void EspUsbHost::get_device_status()
{
    const char *TAG = "get_device_status";

    if (!EspUsbHost::deviceConnected)
    {
        return;
    }

    usb_transfer_t *transfer;
    esp_err_t err = usb_host_transfer_alloc(8 + 2, 0, &transfer);
    if (err != ESP_OK)
    {
        if (LOG_LEVEL_WARN <= EspUsbHost::current_log_level)
        {
            log(LOG_LEVEL_WARN, "%s: Warn: usb_host_transfer_alloc() err=%X", TAG, err);
        }
        return;
    }

    transfer->num_bytes = 8 + 2;
    transfer->data_buffer[0] = USB_BM_REQUEST_TYPE_DIR_IN | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_DEVICE;
    transfer->data_buffer[1] = USB_B_REQUEST_GET_STATUS;
    transfer->data_buffer[2] = 0x00;
    transfer->data_buffer[3] = 0x00;
    transfer->data_buffer[4] = 0x00;
    transfer->data_buffer[5] = 0x00;
    transfer->data_buffer[6] = 0x02;
    transfer->data_buffer[7] = 0x00;

    transfer->device_handle = deviceHandle;
    transfer->bEndpointAddress = 0x00;

    transfer->callback = [](usb_transfer_t *transfer)
    {
        const char *TAG = "get_device_status_callback";
        EspUsbHost *usbHost = static_cast<EspUsbHost *>(transfer->context);

        if (transfer->status == USB_TRANSFER_STATUS_COMPLETED)
        {
            uint16_t status = (transfer->data_buffer[9] << 8) | transfer->data_buffer[8];
            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                usbHost->log(LOG_LEVEL_INFO, "%s: Inform: Device status: %X", TAG, status);
            }

            if (status & (1 << USB_FEATURE_SELECTOR_REMOTE_WAKEUP))
            {
                if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
                {
                    usbHost->log(LOG_LEVEL_INFO, "%s: Inform: Remote Wakeup is enabled.", TAG);
                }
            }
            else
            {
                if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
                {
                    usbHost->log(LOG_LEVEL_INFO, "%s: Inform: Remote Wakeup is disabled.", TAG);
                }
            }
        }
        else
        {
            if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
            {
                usbHost->log(LOG_LEVEL_ERROR, "%s: Error: GET_STATUS transfer failed with status=%X", TAG, transfer->status);
            }
        }

        usb_host_transfer_free(transfer);
    };

    transfer->context = this;

    err = usb_host_transfer_submit_control(clientHandle, transfer);
    if (err != ESP_OK)
    {
        if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
        {
            log(LOG_LEVEL_ERROR, "%s: Error: usb_host_transfer_submit_control() err=%X", TAG, err);
        }
        usb_host_transfer_free(transfer);
        return;
    }
}

void EspUsbHost::suspend_device()
{
    const char *TAG = "suspend_device";

    usb_transfer_t *transfer;
    esp_err_t err = usb_host_transfer_alloc(8 + 1, 0, &transfer);
    if (err != ESP_OK)
    {
        if (LOG_LEVEL_WARN <= EspUsbHost::current_log_level)
        {
            log(LOG_LEVEL_WARN, "%s: Warn: usb_host_transfer_alloc() err=%X", TAG, err);
        }
        return;
    }

    transfer->num_bytes = 8;
    transfer->data_buffer[0] = USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_DEVICE;
    transfer->data_buffer[1] = USB_B_REQUEST_SET_FEATURE;
    transfer->data_buffer[2] = USB_FEATURE_SELECTOR_REMOTE_WAKEUP;
    transfer->data_buffer[3] = 0x00;
    transfer->data_buffer[4] = 0x00;
    transfer->data_buffer[5] = 0x00;
    transfer->data_buffer[6] = 0x00;
    transfer->data_buffer[7] = 0x00;

    transfer->device_handle = deviceHandle;
    transfer->bEndpointAddress = 0x00;
    transfer->callback = _onReceiveControl;
    transfer->context = this;

    err = usb_host_transfer_submit_control(clientHandle, transfer);
    if (err != ESP_OK)
    {

        if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
        {
            log(LOG_LEVEL_ERROR, "%s: Error: usb_host_transfer_submit_control() err=%X", TAG, err);
        }
        usb_host_transfer_free(transfer);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    deviceSuspended = true;

    if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
    {
        log(LOG_LEVEL_INFO, "%s: Inform: Device suspended successfully.", TAG);
    }

    get_device_status();
}

void EspUsbHost::resume_device()
{
    const char *TAG = "resume_device";

    if (!EspUsbHost::deviceConnected)
    {
        return;
    }

    usb_transfer_t *transfer;
    esp_err_t err = usb_host_transfer_alloc(8 + 1, 0, &transfer);
    if (err != ESP_OK)
    {
        if (LOG_LEVEL_WARN <= EspUsbHost::current_log_level)
        {
            log(LOG_LEVEL_WARN, "%s: Warn: usb_host_transfer_alloc() err=%X", TAG, err);
        }
        return;
    }

    transfer->num_bytes = 8;
    transfer->data_buffer[0] = USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_DEVICE;
    transfer->data_buffer[1] = USB_B_REQUEST_CLEAR_FEATURE;
    transfer->data_buffer[2] = USB_FEATURE_SELECTOR_REMOTE_WAKEUP;
    transfer->data_buffer[3] = 0x00;
    transfer->data_buffer[4] = 0x00;
    transfer->data_buffer[5] = 0x00;
    transfer->data_buffer[6] = 0x00;
    transfer->data_buffer[7] = 0x00;

    transfer->device_handle = deviceHandle;
    transfer->bEndpointAddress = 0x00;
    transfer->callback = _onReceiveControl;
    transfer->context = this;

    err = usb_host_transfer_submit_control(clientHandle, transfer);
    if (err != ESP_OK)
    {

        if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
        {
            log(LOG_LEVEL_ERROR, "%s: Error: usb_host_transfer_submit_control() err=%X", TAG, err);
        }
        usb_host_transfer_free(transfer);
        return;
    }

    deviceSuspended = false;

    if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
    {
        log(LOG_LEVEL_INFO, "%s: Inform: Device resumed successfully.", TAG);
    }

    get_device_status();
}

String EspUsbHost::getUsbDescString(const usb_str_desc_t *str_desc)
{
    String str = "";
    if (str_desc == NULL)
    {
        return str;
    }

    for (int i = 0; i < str_desc->bLength / 2; i++)
    {
        if (str_desc->wData[i] > 0xFF)
        {
            continue;
        }
        str += char(str_desc->wData[i]);
    }
    return str;
}

void EspUsbHost::onConfig(const uint8_t bDescriptorType, const uint8_t *p)
{
    static uint8_t currentInterfaceNumber;

    switch (bDescriptorType)
    {
    case USB_DEVICE_DESC:
    {
        // Log the raw bytes first
        logRawBytes(p, p[0], "USB_DEVICE_DESC Raw Data");

        const usb_device_desc_t *dev_desc = (const usb_device_desc_t *)p;

        // Assign each field individually to ensure type compatibility
        descriptor_device.bLength = dev_desc->bLength;
        descriptor_device.bDescriptorType = dev_desc->bDescriptorType;
        descriptor_device.bcdUSB = dev_desc->bcdUSB;
        descriptor_device.bDeviceClass = dev_desc->bDeviceClass;
        descriptor_device.bDeviceSubClass = dev_desc->bDeviceSubClass;
        descriptor_device.bDeviceProtocol = dev_desc->bDeviceProtocol;
        descriptor_device.bMaxPacketSize0 = dev_desc->bMaxPacketSize0;
        descriptor_device.idVendor = dev_desc->idVendor;
        descriptor_device.idProduct = dev_desc->idProduct;
        descriptor_device.bcdDevice = dev_desc->bcdDevice;
        descriptor_device.iManufacturer = dev_desc->iManufacturer;
        descriptor_device.iProduct = dev_desc->iProduct;
        descriptor_device.iSerialNumber = dev_desc->iSerialNumber;
        descriptor_device.bNumConfigurations = dev_desc->bNumConfigurations;

        // Log the device descriptor
        logDeviceDescriptor(*dev_desc);
        break;
    }

    case USB_STRING_DESC:
    {
        // Log the raw bytes first
        logRawBytes(p, p[0], "USB_STRING_DESC Raw Data");

        const usb_standard_desc_t *desc = (const usb_standard_desc_t *)p;
        usb_string_descriptor_t usbStringDescriptor;
        usbStringDescriptor.bLength = desc->bLength;
        usbStringDescriptor.bDescriptorType = desc->bDescriptorType;
        usbStringDescriptor.data = "";

        for (int i = 0; i < (desc->bLength - 2); i++)
        {
            if (desc->val[i] < 16)
            {
                usbStringDescriptor.data += "0";
            }
            usbStringDescriptor.data += String(desc->val[i], HEX) + " ";
        }
        // Log the string descriptor
        logStringDescriptor(usbStringDescriptor);
        break;
    }

    case USB_INTERFACE_DESC:
    {
        // Log the raw bytes first
        logRawBytes(p, p[0], "USB_INTERFACE_DESC Raw Data");

        const usb_intf_desc_t *intf = (const usb_intf_desc_t *)p;

        // Log the interface descriptor
        logInterfaceDescriptor(*intf);

        if (interfaceCounter < MAX_INTERFACE_DESCRIPTORS)
        {
            this->claim_err = usb_host_interface_claim(this->clientHandle, this->deviceHandle, intf->bInterfaceNumber, intf->bAlternateSetting);
            if (this->claim_err != ESP_OK)
            {
                if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
                {
                    log(LOG_LEVEL_ERROR, "USB: Error: usb_host_interface_claim() err=%x", this->claim_err);
                }
            }
            else
            {
                if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
                {
                    log(LOG_LEVEL_INFO, "USB: Inform: usb_host_interface_claim() ESP_OK");
                }
                this->usbInterface[this->usbInterfaceSize] = intf->bInterfaceNumber;
                this->usbInterfaceSize++;

                currentInterfaceNumber = intf->bInterfaceNumber;
                endpoint_data_list[currentInterfaceNumber].bInterfaceNumber = intf->bInterfaceNumber;
                endpoint_data_list[currentInterfaceNumber].bInterfaceClass = intf->bInterfaceClass;
                endpoint_data_list[currentInterfaceNumber].bInterfaceSubClass = intf->bInterfaceSubClass;
                endpoint_data_list[currentInterfaceNumber].bInterfaceProtocol = intf->bInterfaceProtocol;
                endpoint_data_list[currentInterfaceNumber].bCountryCode = 0; // FIX THIS !!
            }

            interfaceCounter++;
        }

        break;
    }

    case USB_ENDPOINT_DESC:
    {
        // Log the raw bytes first
        logRawBytes(p, p[0], "USB_ENDPOINT_DESC Raw Data");

        const usb_ep_desc_t *ep_desc = (const usb_ep_desc_t *)p;

        // Log the endpoint descriptor
        logEndpointDescriptor(*ep_desc);

        if (endpointCounter < MAX_ENDPOINT_DESCRIPTORS)
        {
            endpoint_descriptors[endpointCounter].bLength = ep_desc->bLength;
            endpoint_descriptors[endpointCounter].bDescriptorType = ep_desc->bDescriptorType;
            endpoint_descriptors[endpointCounter].bEndpointAddress = ep_desc->bEndpointAddress;
            endpoint_descriptors[endpointCounter].endpointID = USB_EP_DESC_GET_EP_NUM(ep_desc);
            endpoint_descriptors[endpointCounter].direction = USB_EP_DESC_GET_EP_DIR(ep_desc) ? "IN" : "OUT";
            endpoint_descriptors[endpointCounter].bmAttributes = ep_desc->bmAttributes;
            endpoint_descriptors[endpointCounter].attributes =
                (ep_desc->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_CONTROL ? "CTRL" : (ep_desc->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_ISOC ? "ISOC"
                                                                                                                   : (ep_desc->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_BULK   ? "BULK"
                                                                                                                   : (ep_desc->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_INT    ? "Interrupt"
                                                                                                                                                                                                                : "";

            endpoint_descriptors[endpointCounter].wMaxPacketSize = ep_desc->wMaxPacketSize;
            endpoint_descriptors[endpointCounter].bInterval = ep_desc->bInterval;

            if (this->claim_err != ESP_OK)
            {
                if (LOG_LEVEL_WARN <= EspUsbHost::current_log_level)
                {
                    log(LOG_LEVEL_WARN, "USB: Warn: claim_err encountered, skipping further processing");
                }
                return;
            }

            uint8_t ep_num = USB_EP_DESC_GET_EP_NUM(ep_desc);
            endpoint_data_list[ep_num].bInterfaceNumber = endpoint_data_list[currentInterfaceNumber].bInterfaceNumber;
            endpoint_data_list[ep_num].bInterfaceClass = endpoint_data_list[currentInterfaceNumber].bInterfaceClass;
            endpoint_data_list[ep_num].bInterfaceSubClass = endpoint_data_list[currentInterfaceNumber].bInterfaceSubClass;
            endpoint_data_list[ep_num].bInterfaceProtocol = endpoint_data_list[currentInterfaceNumber].bInterfaceProtocol;
            endpoint_data_list[ep_num].bCountryCode = endpoint_data_list[currentInterfaceNumber].bCountryCode;

            if ((ep_desc->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) != USB_BM_ATTRIBUTES_XFER_INT)
            {
                if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
                {
                    log(LOG_LEVEL_ERROR, "USB: Error: Unsupported transfer type, bmAttributes=%x", ep_desc->bmAttributes);
                }
                return;
            }

            if (ep_desc->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK)
            {
                esp_err_t err = usb_host_transfer_alloc(ep_desc->wMaxPacketSize + 1, 0, &this->usbTransfer[this->usbTransferSize]);
                if (err != ESP_OK)
                {
                    this->usbTransfer[this->usbTransferSize] = NULL;
                    if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
                    {
                        log(LOG_LEVEL_ERROR, "USB: Error: usb_host_transfer_alloc() failed with err=%x", err);
                    }
                    return;
                }
                else
                {
                    if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
                    {
                        log(LOG_LEVEL_INFO, "USB: Inform: usb_host_transfer_alloc() successful, data_buffer_size=%d", ep_desc->wMaxPacketSize + 1);
                    }
                }

                this->usbTransfer[this->usbTransferSize]->device_handle = this->deviceHandle;
                this->usbTransfer[this->usbTransferSize]->bEndpointAddress = ep_desc->bEndpointAddress;
                this->usbTransfer[this->usbTransferSize]->callback = this->_onReceive;
                this->usbTransfer[this->usbTransferSize]->context = this;
                this->usbTransfer[this->usbTransferSize]->num_bytes = ep_desc->wMaxPacketSize;
                interval = ep_desc->bInterval;
                isReady = true;
                this->usbTransferSize++;

                if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
                {
                    log(LOG_LEVEL_INFO, "USB: Inform: Preparing to submit transfer for endpoint 0x%x", ep_desc->bEndpointAddress);
                }

                err = usb_host_transfer_submit(this->usbTransfer[this->usbTransferSize - 1]);
                if (err != ESP_OK)
                {
                    if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
                    {
                        log(LOG_LEVEL_ERROR, "USB: Error: usb_host_transfer_submit() failed with err=%x", err);
                    }
                }
            }

            endpointCounter++;
        }

        break;
    }

    case USB_INTERFACE_ASSOC_DESC:
    {
        // Log the raw bytes first
        logRawBytes(p, p[0], "USB_INTERFACE_ASSOC_DESC Raw Data");

        const usb_iad_desc_t *iad_desc = (const usb_iad_desc_t *)p;

        logInterfaceAssociationDescriptor(*iad_desc);

        descriptor_interface_association.bLength = iad_desc->bLength;
        descriptor_interface_association.bDescriptorType = iad_desc->bDescriptorType;
        descriptor_interface_association.bFirstInterface = iad_desc->bFirstInterface;
        descriptor_interface_association.bInterfaceCount = iad_desc->bInterfaceCount;
        descriptor_interface_association.bFunctionClass = iad_desc->bFunctionClass;
        descriptor_interface_association.bFunctionSubClass = iad_desc->bFunctionSubClass;
        descriptor_interface_association.bFunctionProtocol = iad_desc->bFunctionProtocol;
        descriptor_interface_association.iFunction = iad_desc->iFunction;

        break;
    }

    case USB_HID_DESC:
    {
        // Log the raw bytes first
        logRawBytes(p, p[0], "USB_HID_DESC Raw Data");

        const tusb_hid_descriptor_hid_t *hid_desc = (const tusb_hid_descriptor_hid_t *)p;

        logHIDDescriptor(*hid_desc);

        if (hidDescriptorCounter < MAX_HID_DESCRIPTORS)
        {
            hid_descriptors[hidDescriptorCounter].bLength = hid_desc->bLength;
            hid_descriptors[hidDescriptorCounter].bDescriptorType = hid_desc->bDescriptorType;
            hid_descriptors[hidDescriptorCounter].bcdHID = hid_desc->bcdHID;
            hid_descriptors[hidDescriptorCounter].bCountryCode = hid_desc->bCountryCode;
            hid_descriptors[hidDescriptorCounter].bNumDescriptors = hid_desc->bNumDescriptors;
            hid_descriptors[hidDescriptorCounter].bReportType = hid_desc->bReportType;
            hid_descriptors[hidDescriptorCounter].wReportLength = hid_desc->wReportLength;
            endpoint_data_list[currentInterfaceNumber].bCountryCode = hid_descriptors[hidDescriptorCounter].bCountryCode;

            submitControl(0x81, 0x00, 0x22, currentInterfaceNumber, hid_descriptors[hidDescriptorCounter].wReportLength);

            hidDescriptorCounter++;
        }

        break;
    }
    case USB_CONFIGURATION_DESC:
    {
        // Log the raw bytes first
        logRawBytes(p, p[0], "USB_CONFIGURATION_DESC Raw Data");

        const usb_config_desc_t *config_desc = (const usb_config_desc_t *)p;

        // Log the configuration descriptor fields
        configurationDescriptor.bLength = config_desc->bLength;
        configurationDescriptor.bDescriptorType = config_desc->bDescriptorType;
        configurationDescriptor.wTotalLength = config_desc->wTotalLength;
        configurationDescriptor.bNumInterfaces = config_desc->bNumInterfaces;
        configurationDescriptor.bConfigurationValue = config_desc->bConfigurationValue;
        configurationDescriptor.iConfiguration = config_desc->iConfiguration;
        configurationDescriptor.bmAttributes = config_desc->bmAttributes;
        configurationDescriptor.bMaxPower = config_desc->bMaxPower;

        break;
    }

    default:
    {
        // Log the raw bytes first
        logRawBytes(p, p[0], "Unknown Descriptor Raw Data");

        const usb_standard_desc_t *desc = (const usb_standard_desc_t *)p;

        logUnknownDescriptor(*desc);

        if (unknownDescriptorCounter < MAX_UNKNOWN_DESCRIPTORS)
        {
            unknown_descriptors[unknownDescriptorCounter].bLength = desc->bLength;
            unknown_descriptors[unknownDescriptorCounter].bDescriptorType = desc->bDescriptorType;
            unknown_descriptors[unknownDescriptorCounter].data = "";

            for (int i = 0; i < (desc->bLength - 2); i++)
            {
                if (desc->val[i] < 16)
                {
                    unknown_descriptors[unknownDescriptorCounter].data += "0";
                }
                unknown_descriptors[unknownDescriptorCounter].data += String(desc->val[i], HEX) + " ";
            }

            unknownDescriptorCounter++;
        }
        break;
    }
    }
}

void EspUsbHost::logDeviceDescriptor(const usb_device_desc_t &dev_desc)
{
    if (LOG_LEVEL_PARSED > EspUsbHost::current_log_level)
    {
        return;
    }

    // log(LOG_LEVEL_PARSED, "\n==============================\n");
    log(LOG_LEVEL_PARSED, "**USB Device Descriptor**\n");
    log(LOG_LEVEL_PARSED, "Length: %d\n", dev_desc.bLength);
    log(LOG_LEVEL_PARSED, "Descriptor Type: %d\n", dev_desc.bDescriptorType);
    log(LOG_LEVEL_PARSED, "USB Version: %04X\n", dev_desc.bcdUSB);
    log(LOG_LEVEL_PARSED, "Device Class: %d\n", dev_desc.bDeviceClass);
    log(LOG_LEVEL_PARSED, "Device SubClass: %d\n", dev_desc.bDeviceSubClass);
    log(LOG_LEVEL_PARSED, "Device Protocol: %d\n", dev_desc.bDeviceProtocol);
    log(LOG_LEVEL_PARSED, "Max Packet Size: %d\n", dev_desc.bMaxPacketSize0);
    log(LOG_LEVEL_PARSED, "Vendor ID: %04X\n", dev_desc.idVendor);
    log(LOG_LEVEL_PARSED, "Product ID: %04X\n", dev_desc.idProduct);
    log(LOG_LEVEL_PARSED, "Device Version: %04X\n", dev_desc.bcdDevice);
    log(LOG_LEVEL_PARSED, "Manufacturer Index: %d\n", dev_desc.iManufacturer);
    log(LOG_LEVEL_PARSED, "Product Index: %d\n", dev_desc.iProduct);
    log(LOG_LEVEL_PARSED, "Serial Number Index: %d\n", dev_desc.iSerialNumber);
    log(LOG_LEVEL_PARSED, "Num Configurations: %d\n", dev_desc.bNumConfigurations);
}

void EspUsbHost::logStringDescriptor(const usb_string_descriptor_t &str_desc)
{
    if (LOG_LEVEL_PARSED > EspUsbHost::current_log_level)
    {
        return;
    }

    // log(LOG_LEVEL_PARSED, "\n==============================\n");
    log(LOG_LEVEL_PARSED, "**USB String Descriptor**\n");
    log(LOG_LEVEL_PARSED, "Length: %d\n", str_desc.bLength);
    log(LOG_LEVEL_PARSED, "Descriptor Type: %d\n", str_desc.bDescriptorType);
    log(LOG_LEVEL_PARSED, "String: %s\n", str_desc.data.c_str());
}

void EspUsbHost::logInterfaceDescriptor(const usb_intf_desc_t &intf)
{
    if (LOG_LEVEL_PARSED > EspUsbHost::current_log_level)
    {
        return;
    }

    //   log(LOG_LEVEL_PARSED, "\n==============================\n");
    log(LOG_LEVEL_PARSED, "**USB Interface Descriptor**\n");
    log(LOG_LEVEL_PARSED, "Interface Number: %d\n", intf.bInterfaceNumber);
    log(LOG_LEVEL_PARSED, "Alternate Setting: %d\n", intf.bAlternateSetting);
    log(LOG_LEVEL_PARSED, "Num Endpoints: %d\n", intf.bNumEndpoints);
    log(LOG_LEVEL_PARSED, "Interface Class: %d\n", intf.bInterfaceClass);
    log(LOG_LEVEL_PARSED, "Interface SubClass: %d\n", intf.bInterfaceSubClass);
    log(LOG_LEVEL_PARSED, "Interface Protocol: %d\n", intf.bInterfaceProtocol);
    log(LOG_LEVEL_PARSED, "Interface String Index: %d\n", intf.iInterface);
}

void EspUsbHost::logEndpointDescriptor(const usb_ep_desc_t &ep_desc)
{
    if (LOG_LEVEL_PARSED > EspUsbHost::current_log_level)
    {
        return;
    }

    //   log(LOG_LEVEL_PARSED, "\n==============================\n");
    log(LOG_LEVEL_PARSED, "**USB Endpoint Descriptor**\n");
    log(LOG_LEVEL_PARSED, "Endpoint Address: 0x%02X\n", ep_desc.bEndpointAddress);
    log(LOG_LEVEL_PARSED, "Attributes: 0x%02X\n", ep_desc.bmAttributes);
    log(LOG_LEVEL_PARSED, "Max Packet Size: %d\n", ep_desc.wMaxPacketSize);
    log(LOG_LEVEL_PARSED, "Interval: %d\n", ep_desc.bInterval);
    log(LOG_LEVEL_PARSED, "Direction: %s\n", (ep_desc.bEndpointAddress & 0x80) ? "IN" : "OUT");
    log(LOG_LEVEL_PARSED, "Transfer Type: %s\n",
        (ep_desc.bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_CONTROL ? "Control" : (ep_desc.bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_ISOC ? "Isochronous"
                                                                                                             : (ep_desc.bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_BULK   ? "Bulk"
                                                                                                             : (ep_desc.bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_INT    ? "Interrupt"
                                                                                                                                                                                                         : "Unknown");
}

void EspUsbHost::logInterfaceAssociationDescriptor(const usb_iad_desc_t &iad_desc)
{
    if (LOG_LEVEL_PARSED > EspUsbHost::current_log_level)
    {
        return;
    }

    //  log(LOG_LEVEL_PARSED, "\n==============================\n");
    log(LOG_LEVEL_PARSED, "**USB Interface Association Descriptor**\n");
    log(LOG_LEVEL_PARSED, "Length: %d\n", iad_desc.bLength);
    log(LOG_LEVEL_PARSED, "Descriptor Type: %d\n", iad_desc.bDescriptorType);
    log(LOG_LEVEL_PARSED, "First Interface: %d\n", iad_desc.bFirstInterface);
    log(LOG_LEVEL_PARSED, "Interface Count: %d\n", iad_desc.bInterfaceCount);
    log(LOG_LEVEL_PARSED, "Function Class: %d\n", iad_desc.bFunctionClass);
    log(LOG_LEVEL_PARSED, "Function SubClass: %d\n", iad_desc.bFunctionSubClass);
    log(LOG_LEVEL_PARSED, "Function Protocol: %d\n", iad_desc.bFunctionProtocol);
    log(LOG_LEVEL_PARSED, "Function String Index: %d\n", iad_desc.iFunction);
}

void EspUsbHost::logHIDDescriptor(const tusb_hid_descriptor_hid_t &hid_desc)
{
    if (LOG_LEVEL_PARSED > EspUsbHost::current_log_level)
    {
        return;
    }

    //  log(LOG_LEVEL_PARSED, "\n==============================\n");
    log(LOG_LEVEL_PARSED, "**USB HID Descriptor**\n");
    log(LOG_LEVEL_PARSED, "Length: %d\n", hid_desc.bLength);
    log(LOG_LEVEL_PARSED, "Descriptor Type: %d\n", hid_desc.bDescriptorType);
    log(LOG_LEVEL_PARSED, "HID Class Spec Version: %04X\n", hid_desc.bcdHID);
    log(LOG_LEVEL_PARSED, "Country Code: %d\n", hid_desc.bCountryCode);
    log(LOG_LEVEL_PARSED, "Num Descriptors: %d\n", hid_desc.bNumDescriptors);
    log(LOG_LEVEL_PARSED, "Descriptor Type: %d\n", hid_desc.bReportType);
    log(LOG_LEVEL_PARSED, "Descriptor Length: %d\n", hid_desc.wReportLength);
}

void EspUsbHost::logConfigurationDescriptor(const UsbConfigurationDescriptor &config_desc)
{
    if (LOG_LEVEL_PARSED > EspUsbHost::current_log_level)
    {
        return;
    }

    log(LOG_LEVEL_PARSED, "**Configuration Descriptor**\n");
    log(LOG_LEVEL_PARSED, "Length: %d\n", config_desc.bLength);
    log(LOG_LEVEL_PARSED, "Descriptor Type: %d\n", config_desc.bDescriptorType);
    log(LOG_LEVEL_PARSED, "Total Length: %d\n", config_desc.wTotalLength);
    log(LOG_LEVEL_PARSED, "Number of Interfaces: %d\n", config_desc.bNumInterfaces);
    log(LOG_LEVEL_PARSED, "Configuration Value: %d\n", config_desc.bConfigurationValue);
    log(LOG_LEVEL_PARSED, "Configuration Index: %d\n", config_desc.iConfiguration);
    log(LOG_LEVEL_PARSED, "Attributes: 0x%02X\n", config_desc.bmAttributes);
    log(LOG_LEVEL_PARSED, "Max Power: %d (in 2mA units, so %dmA)\n", config_desc.bMaxPower, config_desc.bMaxPower * 2);
}

void EspUsbHost::logUnknownDescriptor(const usb_standard_desc_t &desc)
{
    if (LOG_LEVEL_PARSED > EspUsbHost::current_log_level)
    {
        return;
    }
    //   log(LOG_LEVEL_PARSED, "\n==============================\n");
    log(LOG_LEVEL_PARSED, "**Unknown Descriptor**\n");
    log(LOG_LEVEL_PARSED, "Length: %d\n", desc.bLength);
    log(LOG_LEVEL_PARSED, "Descriptor Type: %d\n", desc.bDescriptorType);

    std::ostringstream rawDataStream;
    for (int i = 0; i < desc.bLength; ++i)
    {
        rawDataStream << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                      << static_cast<int>(desc.val[i]) << " ";
    }
}

void EspUsbHost::_clientEventCallback(const usb_host_client_event_msg_t *eventMsg, void *arg)
{
    EspUsbHost *usbHost = static_cast<EspUsbHost *>(arg);
    esp_err_t err;

    switch (eventMsg->event)
    {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
    {
        if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
        {
            usbHost->log(LOG_LEVEL_INFO, "Inform: DEVICE CONNECTED");
        }

        EspUsbHost::deviceConnected = true;
        usbHost->endpointCounter = 0;
        usbHost->interfaceCounter = 0;
        usbHost->hidDescriptorCounter = 0;
        usbHost->unknownDescriptorCounter = 0;
        memset(usbHost->endpoint_data_list, 0, sizeof(usbHost->endpoint_data_list));

        if (LOG_LEVEL_DEBUG <= EspUsbHost::current_log_level)
        {
            usbHost->log(LOG_LEVEL_DEBUG, "Debug: Event message raw bytes:");
            for (size_t i = 0; i < sizeof(usb_host_client_event_msg_t); ++i)
            {
                usbHost->log(LOG_LEVEL_DEBUG, "0x%02X ", ((uint8_t *)eventMsg)[i]);
            }
            usbHost->log(LOG_LEVEL_DEBUG, "\n");
        }

        err = usb_host_device_open(usbHost->clientHandle, eventMsg->new_dev.address, &usbHost->deviceHandle);
        if (err != ESP_OK)
        {
            if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
            {
                usbHost->log(LOG_LEVEL_ERROR, "Error: Failed to open device with address %d", eventMsg->new_dev.address);
            }
            return;
        }

        if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
        {
            usbHost->log(LOG_LEVEL_INFO, "Inform: Device opened successfully");
        }

        usb_device_info_t dev_info;
        err = usb_host_device_info(usbHost->deviceHandle, &dev_info);
        if (err == ESP_OK)
        {
            usbHost->logRawBytes((uint8_t *)&dev_info, sizeof(usb_device_info_t), "USB Device Information Raw Data");
            usbHost->device_info.speed = dev_info.speed;
            usbHost->device_info.dev_addr = dev_info.dev_addr;
            usbHost->device_info.vMaxPacketSize0 = dev_info.bMaxPacketSize0;
            usbHost->device_info.bConfigurationValue = dev_info.bConfigurationValue;
            strcpy(usbHost->device_info.str_desc_manufacturer, getUsbDescString(dev_info.str_desc_manufacturer).c_str());
            strcpy(usbHost->device_info.str_desc_product, getUsbDescString(dev_info.str_desc_product).c_str());
            strcpy(usbHost->device_info.str_desc_serial_num, getUsbDescString(dev_info.str_desc_serial_num).c_str());

            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                usbHost->log(LOG_LEVEL_INFO, "Inform: Device info retrieved successfully");
            }

            if (LOG_LEVEL_DEBUG <= EspUsbHost::current_log_level)
            {
                usbHost->log(LOG_LEVEL_DEBUG, "Debug: Device info raw bytes:");
                for (size_t i = 0; i < sizeof(usb_device_info_t); ++i)
                {
                    usbHost->log(LOG_LEVEL_DEBUG, "0x%02X ", ((uint8_t *)&dev_info)[i]);
                }
                usbHost->log(LOG_LEVEL_DEBUG, "\n");
            }
        }
        else
        {
            if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
            {
                usbHost->log(LOG_LEVEL_ERROR, "Error: Failed to retrieve device info, err=%d", err);
            }
        }

        const usb_device_desc_t *dev_desc;
        err = usb_host_get_device_descriptor(usbHost->deviceHandle, &dev_desc);
        if (err == ESP_OK)
        {
            usbHost->descriptor_device.bLength = dev_desc->bLength;
            usbHost->descriptor_device.bDescriptorType = dev_desc->bDescriptorType;
            usbHost->descriptor_device.bcdUSB = dev_desc->bcdUSB;
            usbHost->descriptor_device.bDeviceClass = dev_desc->bDeviceClass;
            usbHost->descriptor_device.bDeviceSubClass = dev_desc->bDeviceSubClass;
            usbHost->descriptor_device.bDeviceProtocol = dev_desc->bDeviceProtocol;
            usbHost->descriptor_device.bMaxPacketSize0 = dev_desc->bMaxPacketSize0;
            usbHost->descriptor_device.idVendor = dev_desc->idVendor;
            usbHost->descriptor_device.idProduct = dev_desc->idProduct;
            usbHost->descriptor_device.bcdDevice = dev_desc->bcdDevice;
            usbHost->descriptor_device.iManufacturer = dev_desc->iManufacturer;
            usbHost->descriptor_device.iProduct = dev_desc->iProduct;
            usbHost->descriptor_device.iSerialNumber = dev_desc->iSerialNumber;
            usbHost->descriptor_device.bNumConfigurations = dev_desc->bNumConfigurations;

            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                usbHost->log(LOG_LEVEL_INFO, "Inform: Device descriptor retrieved successfully");
            }

            if (LOG_LEVEL_DEBUG <= EspUsbHost::current_log_level)
            {
                usbHost->log(LOG_LEVEL_DEBUG, "Debug: Device descriptor raw bytes:");
                for (size_t i = 0; i < sizeof(usb_device_desc_t); ++i)
                {
                    usbHost->log(LOG_LEVEL_DEBUG, "0x%02X ", ((uint8_t *)dev_desc)[i]);
                }
                usbHost->log(LOG_LEVEL_DEBUG, "\n");
            }
        }
        else
        {
            if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
            {
                usbHost->log(LOG_LEVEL_ERROR, "Error: Failed to retrieve device descriptor, err=%d", err);
            }
        }

        const usb_config_desc_t *config_desc;
        err = usb_host_get_active_config_descriptor(usbHost->deviceHandle, &config_desc);
        if (err == ESP_OK)
        {
            usbHost->descriptor_configuration.bLength = config_desc->bLength;
            usbHost->descriptor_configuration.bDescriptorType = config_desc->bDescriptorType;
            usbHost->descriptor_configuration.wTotalLength = config_desc->wTotalLength;
            usbHost->descriptor_configuration.bNumInterfaces = config_desc->bNumInterfaces;
            usbHost->descriptor_configuration.bConfigurationValue = config_desc->bConfigurationValue;
            usbHost->descriptor_configuration.iConfiguration = config_desc->iConfiguration;
            usbHost->descriptor_configuration.bmAttributes = config_desc->bmAttributes;
            usbHost->descriptor_configuration.bMaxPower = config_desc->bMaxPower * 2;

            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                usbHost->log(LOG_LEVEL_INFO, "Inform: Configuration descriptor retrieved successfully");
            }

            if (LOG_LEVEL_DEBUG <= EspUsbHost::current_log_level)
            {
                usbHost->log(LOG_LEVEL_DEBUG, "Debug: Configuration descriptor raw bytes:");
                for (size_t i = 0; i < config_desc->wTotalLength; ++i)
                {
                    usbHost->log(LOG_LEVEL_DEBUG, "0x%02X ", ((uint8_t *)config_desc)[i]);
                }
                usbHost->log(LOG_LEVEL_DEBUG, "\n");
            }

            usbHost->_configCallback(config_desc);
        }
        else
        {
            if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
            {
                usbHost->log(LOG_LEVEL_ERROR, "Error: Failed to retrieve configuration descriptor, err=%d", err);
            }
        }

        break;
    }

    case USB_HOST_CLIENT_EVENT_DEV_GONE:
    {
        if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
        {
            usbHost->log(LOG_LEVEL_INFO, "Inform: Device disconnected");
        }

        usbHost->isReady = false;
        EspUsbHost::deviceConnected = false;
        deviceMouseReady = false;

        if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
        {
            usbHost->log(LOG_LEVEL_INFO, "Inform: Clearing USB transfers");
        }

        // Clear and free USB transfers
        for (int i = 0; i < usbHost->usbTransferSize; i++)
        {
            if (usbHost->usbTransfer[i] == NULL)
            {
                continue;
            }

            err = usb_host_transfer_free(usbHost->usbTransfer[i]);
            if (err == ESP_OK)
            {
                if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
                {
                    usbHost->log(LOG_LEVEL_INFO, "Inform: Freed USB transfer at index %d", i);
                }
                usbHost->usbTransfer[i] = NULL;
            }
            else
            {
                if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
                {
                    usbHost->log(LOG_LEVEL_ERROR, "Error: Failed to free USB transfer at index %d, err=%d", i, err);
                }
            }
        }
        usbHost->usbTransferSize = 0;

        if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
        {
            usbHost->log(LOG_LEVEL_INFO, "Inform: Releasing USB interfaces");
        }

        // Release interfaces
        for (int i = 0; i < usbHost->usbInterfaceSize; i++)
        {
            err = usb_host_interface_release(usbHost->clientHandle, usbHost->deviceHandle, usbHost->usbInterface[i]);
            if (err == ESP_OK)
            {
                if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
                {
                    usbHost->log(LOG_LEVEL_INFO, "Inform: Released USB interface at index %d", i);
                }
                usbHost->usbInterface[i] = 0;
            }
            else
            {
                if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
                {
                    usbHost->log(LOG_LEVEL_ERROR, "Error: Failed to release USB interface at index %d, err=%d", i, err);
                }
            }
        }
        usbHost->usbInterfaceSize = 0;

        if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
        {
            usbHost->log(LOG_LEVEL_INFO, "Inform: Closing USB device");
        }

        err = usb_host_device_close(usbHost->clientHandle, usbHost->deviceHandle);
        if (err != ESP_OK)
        {
            if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
            {
                usbHost->log(LOG_LEVEL_ERROR, "Error: Failed to close device, err=%d", err);
            }
            usbHost->tx_Que("Error closing device\n"); // Use tx_Que instead of Serial1
        }
        else
        {
            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                usbHost->log(LOG_LEVEL_INFO, "Inform: Device closed successfully");
            }
        }

        usbHost->isReady = false;
        usbHost->isClientRegistering = false;

        if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
        {
            usbHost->log(LOG_LEVEL_INFO, "Inform: Notifying system of device disconnection");
        }

        usbHost->onGone(eventMsg);

        if (!usbHost->debugModeActive)
        {
            usbHost->tx_Que("USB_GOODBYE\n");
        }
        else
        {
            usbHost->log(LOG_LEVEL_DEBUG, "Please reinsert USB mouse to begin logging!!");
        }

        usbHost->tx_Que("MOUSE DISCONNECTED\n");

        break;
    }

    default:
        break;
    }
}

void EspUsbHost::_configCallback(const usb_config_desc_t *config_desc)
{
    const uint8_t *p = &config_desc->val[0];
    uint8_t bLength;

    if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
    {
        this->log(LOG_LEVEL_INFO, "Inform: Starting configuration descriptor processing");
    }

    const uint8_t setup[8] = {
        0x80, 0x06, 0x00, 0x02, 0x00, 0x00, (uint8_t)config_desc->wTotalLength, 0x00};

    for (int i = 0; i < config_desc->wTotalLength; i += bLength, p += bLength)
    {
        bLength = *p;

        if ((i + bLength) <= config_desc->wTotalLength)
        {
            const uint8_t bDescriptorType = *(p + 1);

            if (LOG_LEVEL_DEBUG <= EspUsbHost::current_log_level)
            {
                this->log(LOG_LEVEL_DEBUG, "Debug: Processing descriptor of type: 0x%02X", bDescriptorType);
            }

            this->onConfig(bDescriptorType, p);
        }
        else
        {
            if (LOG_LEVEL_WARN <= EspUsbHost::current_log_level)
            {
                this->log(LOG_LEVEL_WARN, "Warn: Descriptor length exceeds total configuration length");
            }
            return;
        }
    }

    if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
    {
        this->log(LOG_LEVEL_INFO, "Inform: Completed configuration descriptor processing");
    }
}

void EspUsbHost::_onReceiveControl(usb_transfer_t *transfer)
{
    EspUsbHost *usbHost = static_cast<EspUsbHost *>(transfer->context);
    if (!usbHost)
    {
        return;
    }
    usbHost->logRawBytes(transfer->data_buffer, transfer->actual_num_bytes, "Control Transfer Response Raw Data");

    bool isMouse = false;
    uint8_t *p = &transfer->data_buffer[8];
    int totalBytes = transfer->actual_num_bytes;

    if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
    {
        usbHost->log(LOG_LEVEL_INFO, "onReceiveControl called with %d bytes", totalBytes);
    }

    if (LOG_LEVEL_DEBUG <= EspUsbHost::current_log_level)
    {
        std::ostringstream rawDataStream;
        for (int i = 0; i < totalBytes; ++i)
        {
            rawDataStream << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(p[i]) << " ";
        }
        usbHost->log(LOG_LEVEL_DEBUG, "Raw data: %s", rawDataStream.str().c_str());
    }

    for (int i = 0; i < totalBytes - 3; i++)
    {
        if (p[i] == 0x05 && p[i + 1] == 0x01 && p[i + 2] == 0x09 && p[i + 3] == 0x02)
        {
            isMouse = true;
            break;
        }
    }

    if (!isMouse)
    {
        if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
        {
            usbHost->log(LOG_LEVEL_INFO, "Device is not a mouse, skipping further processing");
        }
        usb_host_transfer_free(transfer);
        return;
    }

    if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
    {
        usbHost->log(LOG_LEVEL_INFO, "Mouse device detected, parsing HID report descriptor");
    }

    HIDReportDescriptor descriptor = usbHost->parseHIDReportDescriptor(&transfer->data_buffer[8], transfer->actual_num_bytes - 8);

    usb_host_transfer_free(transfer);
}

void EspUsbHost::_onReceive(usb_transfer_t *transfer)
{
    EspUsbHost *usbHost = static_cast<EspUsbHost *>(transfer->context);
    if (!usbHost)
    {
        ESP_LOGE("EspUsbHost", "Error: Context pointer is null in _onReceive");
        usb_host_transfer_free(transfer);
        return;
    }

    static unsigned long lastLogTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastLogTime >= 250)
    {
        usbHost->logRawBytes(transfer->data_buffer, transfer->actual_num_bytes, "Interrupt Transfer Raw Data");
        lastLogTime = currentTime;
    }

    uint8_t endpoint_num = transfer->bEndpointAddress & 0x0F;
    bool has_data = (transfer->actual_num_bytes > 0);

    if (has_data)
    {
        usbHost->last_activity_time = millis();
        if (EspUsbHost::deviceConnected && usbHost->deviceSuspended)
        {
            usbHost->resume_device();
        }
        flashLED();
    }

    if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
    {
        usbHost->log(LOG_LEVEL_INFO, "Received HID report: ");
    }

    if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
    {
        std::ostringstream oss;
        for (int i = 0; i < transfer->actual_num_bytes; i++)
        {
            oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                << static_cast<int>(transfer->data_buffer[i]) << " ";
        }
        usbHost->log(LOG_LEVEL_INFO, "%s", oss.str().c_str());
    }

    for (int i = 0; i < 16; i++)
    {
        if (usbHost->endpoint_data_list[i].bInterfaceClass == USB_CLASS_HID)
        {
            if (usbHost->endpoint_data_list[i].bInterfaceSubClass == HID_SUBCLASS_BOOT &&
                usbHost->endpoint_data_list[i].bInterfaceProtocol == HID_ITF_PROTOCOL_MOUSE)
            {

                static uint8_t last_buttons = 0;
                hid_mouse_report_t report = {};
                report.buttons = transfer->data_buffer[usbHost->HIDReportDesc.buttonStartByte];

                if (usbHost->HIDReportDesc.xAxisSize == 12 && usbHost->HIDReportDesc.yAxisSize == 12)
                {
                    uint8_t xyOffset = usbHost->HIDReportDesc.xAxisStartByte;
                    int16_t xValue = (transfer->data_buffer[xyOffset]) |
                                     ((transfer->data_buffer[xyOffset + 1] & 0x0F) << 8);
                    int16_t yValue = ((transfer->data_buffer[xyOffset + 1] >> 4) & 0x0F) |
                                     (transfer->data_buffer[xyOffset + 2] << 4);

                    report.x = xValue;
                    report.y = yValue;
                    uint8_t wheelOffset = usbHost->HIDReportDesc.wheelStartByte;
                    report.wheel = transfer->data_buffer[wheelOffset];
                }
                else
                {
                    uint8_t xOffset = usbHost->HIDReportDesc.xAxisStartByte;
                    uint8_t yOffset = usbHost->HIDReportDesc.yAxisStartByte;
                    uint8_t wheelOffset = usbHost->HIDReportDesc.wheelStartByte;

                    report.x = transfer->data_buffer[xOffset];
                    report.y = transfer->data_buffer[yOffset];
                    report.wheel = transfer->data_buffer[wheelOffset];
                }

                usbHost->onMouse(report, last_buttons);
                if (report.buttons != last_buttons)
                {
                    usbHost->onMouseButtons(report, last_buttons);
                    last_buttons = report.buttons;
                }
                if (report.x != 0 || report.y != 0 || report.wheel != 0)
                {
                    usbHost->onMouseMove(report);
                }
            }
        }
    }

    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED)
    {
        if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
        {
            usbHost->log(LOG_LEVEL_INFO, "Transfer Completed Successfully: Endpoint=%x", transfer->bEndpointAddress);
        }
    }
    else if (transfer->status == USB_TRANSFER_STATUS_STALL)
    {
        if (LOG_LEVEL_WARN <= EspUsbHost::current_log_level)
        {
            usbHost->log(LOG_LEVEL_WARN, "Transfer STALL Received: Endpoint=%x", transfer->bEndpointAddress);
        }
    }
    else
    {
        if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
        {
            usbHost->log(LOG_LEVEL_ERROR, "Transfer Error or Incomplete: Status=%x, Endpoint=%x", transfer->status, transfer->bEndpointAddress);
        }
    }

    if (!usbHost->deviceSuspended)
    {
        esp_err_t err = usb_host_transfer_submit(transfer);
        if (err != ESP_OK)
        {
            if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
            {
                usbHost->log(LOG_LEVEL_ERROR, "Failed to resubmit transfer: err=%x, Endpoint=%x", err, transfer->bEndpointAddress);
            }
        }
    }
    else
    {
        usb_host_transfer_free(transfer);
    }
}

void EspUsbHost::onMouse(hid_mouse_report_t report, uint8_t last_buttons)
{
    if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
    {
        log(LOG_LEVEL_INFO, "Inform: Mouse event detected");
    }

    if (LOG_LEVEL_DEBUG <= EspUsbHost::current_log_level)
    {
        log(LOG_LEVEL_DEBUG, "Debug: last_buttons=0x%02x(%c%c%c%c%c), buttons=0x%02x(%c%c%c%c%c), x=%d, y=%d, wheel=%d",
            last_buttons,
            (last_buttons & MOUSE_BUTTON_LEFT) ? 'L' : ' ',
            (last_buttons & MOUSE_BUTTON_RIGHT) ? 'R' : ' ',
            (last_buttons & MOUSE_BUTTON_MIDDLE) ? 'M' : ' ',
            (last_buttons & MOUSE_BUTTON_BACKWARD) ? 'B' : ' ',
            (last_buttons & MOUSE_BUTTON_FORWARD) ? 'F' : ' ',
            report.buttons,
            (report.buttons & MOUSE_BUTTON_LEFT) ? 'L' : ' ',
            (report.buttons & MOUSE_BUTTON_RIGHT) ? 'R' : ' ',
            (report.buttons & MOUSE_BUTTON_MIDDLE) ? 'M' : ' ',
            (report.buttons & MOUSE_BUTTON_BACKWARD) ? 'B' : ' ',
            (report.buttons & MOUSE_BUTTON_FORWARD) ? 'F' : ' ',
            report.x,
            report.y,
            report.wheel);
    }
}

void EspUsbHost::onMouseButtons(hid_mouse_report_t report, uint8_t last_buttons)
{
    if (deviceMouseReady)
    {
        if (!(last_buttons & MOUSE_BUTTON_LEFT) && (report.buttons & MOUSE_BUTTON_LEFT))
        {
            tx_Que("km.left(1)\n");
            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                log(LOG_LEVEL_INFO, "Inform: Left mouse button pressed");
            }
        }
        if ((last_buttons & MOUSE_BUTTON_LEFT) && !(report.buttons & MOUSE_BUTTON_LEFT))
        {
            tx_Que("km.left(0)\n");
            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                log(LOG_LEVEL_INFO, "Inform: Left mouse button released");
            }
        }

        if (!(last_buttons & MOUSE_BUTTON_RIGHT) && (report.buttons & MOUSE_BUTTON_RIGHT))
        {
            tx_Que("km.right(1)\n");
            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                log(LOG_LEVEL_INFO, "Inform: Right mouse button pressed");
            }
        }
        if ((last_buttons & MOUSE_BUTTON_RIGHT) && !(report.buttons & MOUSE_BUTTON_RIGHT))
        {
            tx_Que("km.right(0)\n");
            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                log(LOG_LEVEL_INFO, "Inform: Right mouse button released");
            }
        }

        if (!(last_buttons & MOUSE_BUTTON_MIDDLE) && (report.buttons & MOUSE_BUTTON_MIDDLE))
        {
            tx_Que("km.middle(1)\n");
            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                log(LOG_LEVEL_INFO, "Inform: Middle mouse button pressed");
            }
        }
        if ((last_buttons & MOUSE_BUTTON_MIDDLE) && !(report.buttons & MOUSE_BUTTON_MIDDLE))
        {
            tx_Que("km.middle(0)\n");
            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                log(LOG_LEVEL_INFO, "Inform: Middle mouse button released");
            }
        }

        if (!(last_buttons & MOUSE_BUTTON_FORWARD) && (report.buttons & MOUSE_BUTTON_FORWARD))
        {
            tx_Que("km.side1(1)\n");
            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                log(LOG_LEVEL_INFO, "Inform: Forward mouse button pressed");
            }
        }
        if ((last_buttons & MOUSE_BUTTON_FORWARD) && !(report.buttons & MOUSE_BUTTON_FORWARD))
        {
            tx_Que("km.side1(0)\n");
            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                log(LOG_LEVEL_INFO, "Inform: Forward mouse button released");
            }
        }

        if (!(last_buttons & MOUSE_BUTTON_BACKWARD) && (report.buttons & MOUSE_BUTTON_BACKWARD))
        {
            tx_Que("km.side2(1)\n");
            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                log(LOG_LEVEL_INFO, "Inform: Backward mouse button pressed");
            }
        }
        if ((last_buttons & MOUSE_BUTTON_BACKWARD) && !(report.buttons & MOUSE_BUTTON_BACKWARD))
        {
            tx_Que("km.side2(0)\n");
            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                log(LOG_LEVEL_INFO, "Inform: Backward mouse button released");
            }
        }
    }
}

void EspUsbHost::onMouseMove(hid_mouse_report_t report)
{
    if (deviceMouseReady)
    {
        if (report.wheel != 0)
        {
            tx_Que("km.wheel(%d)\n", report.wheel);
            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                log(LOG_LEVEL_INFO, "Inform: Mouse wheel moved, value=%d", report.wheel);
            }
        }
        else
        {
            tx_Que("km.move(%d,%d)\n", report.x, report.y);
            if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
            {
                log(LOG_LEVEL_INFO, "Inform: Mouse moved, x=%d, y=%d", report.x, report.y);
            }
        }
    }
}

esp_err_t EspUsbHost::submitControl(const uint8_t bmRequestType,
                                    const uint8_t bDescriptorIndex,
                                    const uint8_t bDescriptorType,
                                    const uint16_t wInterfaceNumber,
                                    const uint16_t wDescriptorLength)
{
    usb_transfer_t *transfer;
    esp_err_t err = usb_host_transfer_alloc(wDescriptorLength + 8 + 1, 0, &transfer);
    if (err != ESP_OK)
    {
        if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
        {
            log(LOG_LEVEL_ERROR, "Error: usb_host_transfer_alloc() failed with err=%x", err);
        }
        return err;
    }

    transfer->num_bytes = wDescriptorLength + 8;
    transfer->data_buffer[0] = bmRequestType;
    transfer->data_buffer[1] = 0x06;
    transfer->data_buffer[2] = bDescriptorIndex;
    transfer->data_buffer[3] = bDescriptorType;
    transfer->data_buffer[4] = wInterfaceNumber & 0xff;
    transfer->data_buffer[5] = wInterfaceNumber >> 8;
    transfer->data_buffer[6] = wDescriptorLength & 0xff;
    transfer->data_buffer[7] = wDescriptorLength >> 8;

    transfer->device_handle = deviceHandle;
    transfer->bEndpointAddress = 0x00;
    transfer->callback = _onReceiveControl;
    transfer->context = this;
    logRawBytes(transfer->data_buffer, transfer->num_bytes, "Control Transfer Request Raw Data");

    if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
    {
        log(LOG_LEVEL_INFO, "Submitting control transfer, bmRequestType=0x%02x, bDescriptorIndex=0x%02x, bDescriptorType=0x%02x, wInterfaceNumber=0x%04x, wDescriptorLength=%d",
            bmRequestType, bDescriptorIndex, bDescriptorType, wInterfaceNumber, wDescriptorLength);
    }

    err = usb_host_transfer_submit_control(clientHandle, transfer);
    if (err != ESP_OK)
    {
        if (LOG_LEVEL_ERROR <= EspUsbHost::current_log_level)
        {
            log(LOG_LEVEL_ERROR, "Error: usb_host_transfer_submit_control() failed with err=%x", err);
        }
        usb_host_transfer_free(transfer);
    }
    else
    {
        if (LOG_LEVEL_INFO <= EspUsbHost::current_log_level)
        {
            log(LOG_LEVEL_INFO, "Control transfer submitted successfully");
        }
    }

    return err;
}

static uint8_t getItemSize(uint8_t prefix)
{
    return (prefix & 0x03) + 1;
}

static uint8_t getItemType(uint8_t prefix)
{
    return prefix & 0xFC;
}

EspUsbHost::HIDReportDescriptor EspUsbHost::parseHIDReportDescriptor(uint8_t *data, int length)
{
    logRawBytes(data, length, "parsed HID Report Descriptor Raw Data");
    int i = 0;

    ParsedValues parsedValues = {0};

    auto getValue = [](uint8_t *data, int size, bool isSigned) -> int16_t
    {
        int16_t value = 0;
        if (isSigned)
        {
            if (size == 1)
            {
                value = (int8_t)data[0];
            }
            else if (size == 2)
            {
                value = (int16_t)(data[0] | (data[1] << 8));
            }
        }
        else
        {
            if (size == 1)
            {
                value = (uint8_t)data[0];
            }
            else if (size == 2)
            {
                value = (uint16_t)(data[0] | (data[1] << 8));
            }
        }
        return value;
    };

    HIDReportDescriptor localHIDReportDesc = {0};

    while (i < length)
    {
        uint8_t prefix = data[i];
        parsedValues.size = (prefix & 0x03);
        parsedValues.size = (parsedValues.size == 3) ? 4 : parsedValues.size;
        uint8_t item = prefix & 0xFC;
        bool isSigned = (item == 0x14 || item == 0x24);
        int16_t value = getValue(data + i + 1, parsedValues.size, isSigned);

        switch (item)
        {
        case 0x04: // USAGE_PAGE
            parsedValues.usagePage = (uint8_t)value;
            break;
        case 0x08: // USAGE
            parsedValues.usage = (uint8_t)value;
            break;
        case 0x84: // REPORT_ID
            parsedValues.reportId = value;
            localHIDReportDesc.reportId = parsedValues.reportId;
            parsedValues.hasReportId = true;
            parsedValues.currentBitOffset += 8;
            break;
        case 0x74: // REPORT_SIZE
            parsedValues.reportSize = value;
            break;
        case 0x94: // REPORT_COUNT
            parsedValues.reportCount = value;
            break;
        case 0x14: // LOGICAL_MINIMUM
            if (parsedValues.size == 1)
            {
                parsedValues.logicalMin8 = (int8_t)value;
            }
            else
            {
                parsedValues.logicalMin16 = value;
            }
            break;
        case 0x24: // LOGICAL_MAXIMUM
            if (parsedValues.size == 1)
            {
                parsedValues.logicalMax8 = (int8_t)value;
            }
            else
            {
                parsedValues.logicalMax = value;
            }
            break;
        case 0xA0: // COLLECTION
            parsedValues.level++;
            parsedValues.collection = value;
            break;
        case 0xC0: // END_COLLECTION
            parsedValues.level--;
            break;
        case 0x80: // INPUT
            // Determine what type of input we are dealing with based on USAGE
            if (parsedValues.usagePage == 0x01 && (parsedValues.usage == 0x30 || parsedValues.usage == 0x31))
            { // X-axis or Y-axis
                if (parsedValues.logicalMax <= 2047)
                {
                    // Handle 12-bit range
                    localHIDReportDesc.xAxisSize = 12;
                    localHIDReportDesc.yAxisSize = 12;
                    localHIDReportDesc.xAxisStartByte = parsedValues.currentBitOffset / 8;
                    parsedValues.currentBitOffset += 12;
                    localHIDReportDesc.yAxisStartByte = parsedValues.currentBitOffset / 8;
                    parsedValues.currentBitOffset += 12;
                }
                else
                {
                    // Handle 8-bit or 16-bit ranges
                    localHIDReportDesc.xAxisSize = localHIDReportDesc.yAxisSize =
                        (parsedValues.logicalMax <= 127) ? 8 : 16;
                    localHIDReportDesc.xAxisStartByte = parsedValues.currentBitOffset / 8;
                    parsedValues.currentBitOffset += localHIDReportDesc.xAxisSize;
                    localHIDReportDesc.yAxisStartByte = parsedValues.currentBitOffset / 8;
                    parsedValues.currentBitOffset += localHIDReportDesc.yAxisSize;
                }
            }
            else if (parsedValues.usagePage == 0x01 && parsedValues.usage == 0x38)
            {
                localHIDReportDesc.wheelSize = (parsedValues.logicalMax <= 127 && parsedValues.logicalMax >= -128) ? 8 : 16;
                localHIDReportDesc.wheelStartByte = parsedValues.currentBitOffset / 8;
                parsedValues.currentBitOffset += localHIDReportDesc.wheelSize;
            }
            else if (parsedValues.usagePage == 0x09 && parsedValues.usage >= 0x01 && parsedValues.usage <= 0x10)
            {
                localHIDReportDesc.buttonSize = parsedValues.reportCount * parsedValues.reportSize;
                localHIDReportDesc.buttonStartByte = parsedValues.currentBitOffset / 8;
                parsedValues.currentBitOffset += localHIDReportDesc.buttonSize;
            }
            break;
        case 0x18:
            parsedValues.usageMinimum = (uint8_t)value;
            break;
        case 0x28:
            parsedValues.usageMaximum = (uint8_t)value;
            break;
        default:
            break;
        }

        i += parsedValues.size + 1;
    }

    HIDReportDesc = localHIDReportDesc;

    logHIDReportDescriptor(HIDReportDesc);

    return HIDReportDesc;
}

void EspUsbHost::logHIDReportDescriptor(const HIDReportDescriptor &desc)
{
    if (LOG_LEVEL_PARSED > EspUsbHost::current_log_level)
    {
        return;
    }

    log(LOG_LEVEL_PARSED, "\n==============================\n");
    log(LOG_LEVEL_PARSED, "**HID Report Descriptor**\n");
    log(LOG_LEVEL_PARSED, "Report ID: %02X\n", desc.reportId);
    log(LOG_LEVEL_PARSED, "Button Bitmap Size: %d\n", desc.buttonSize);
    log(LOG_LEVEL_PARSED, "X-Axis Size: %d\n", desc.xAxisSize);
    log(LOG_LEVEL_PARSED, "Y-Axis Size: %d\n", desc.yAxisSize);
    log(LOG_LEVEL_PARSED, "Wheel Size: %d\n", desc.wheelSize);
    log(LOG_LEVEL_PARSED, "Button Start Byte: %d\n", desc.buttonStartByte);
    log(LOG_LEVEL_PARSED, "X Axis Start Byte: %d\n", desc.xAxisStartByte);
    log(LOG_LEVEL_PARSED, "Y Axis Start Byte: %d\n", desc.yAxisStartByte);
    log(LOG_LEVEL_PARSED, "Wheel Start Byte: %d\n", desc.wheelStartByte);
}

void ledFlashTask(void *parameter)
{
    pinMode(9, OUTPUT);

    while (true)
    {
        if (xSemaphoreTake(ledSemaphore, portMAX_DELAY) == pdTRUE)
        {
            digitalWrite(9, HIGH);
            vTaskDelay(ledFlashTime / portTICK_PERIOD_MS);
            digitalWrite(9, LOW);
            vTaskDelay(ledFlashTime / portTICK_PERIOD_MS);
        }
    }
}

void flashLED()
{
    if (ledSemaphore != NULL)
    {
        xSemaphoreGive(ledSemaphore);
    }
}

void EspUsbHost::sendDeviceInfo()
{
    Serial1.print("USB_sendDeviceInfo:");
    JsonDocument doc;
    doc["speed"] = device_info.speed;
    doc["dev_addr"] = device_info.dev_addr;
    doc["vMaxPacketSize0"] = device_info.vMaxPacketSize0;
    doc["bConfigurationValue"] = device_info.bConfigurationValue;
    doc["str_desc_manufacturer"] = device_info.str_desc_manufacturer;
    doc["str_desc_product"] = device_info.str_desc_product;
    doc["str_desc_serial_num"] = device_info.str_desc_serial_num;
    serializeJson(doc, Serial1);
    Serial1.println();
}

void EspUsbHost::sendDescriptorDevice()
{
    Serial1.print("USB_sendDescriptorDevice:");
    JsonDocument doc;
    doc["bLength"] = descriptor_device.bLength;
    doc["bDescriptorType"] = descriptor_device.bDescriptorType;
    doc["bcdUSB"] = descriptor_device.bcdUSB;
    doc["bDeviceClass"] = descriptor_device.bDeviceClass;
    doc["bDeviceSubClass"] = descriptor_device.bDeviceSubClass;
    doc["bDeviceProtocol"] = descriptor_device.bDeviceProtocol;
    doc["bMaxPacketSize0"] = descriptor_device.bMaxPacketSize0;
    doc["idVendor"] = descriptor_device.idVendor;
    doc["idProduct"] = descriptor_device.idProduct;
    doc["bcdDevice"] = descriptor_device.bcdDevice;
    doc["iManufacturer"] = descriptor_device.iManufacturer;
    doc["iProduct"] = descriptor_device.iProduct;
    doc["iSerialNumber"] = descriptor_device.iSerialNumber;
    doc["bNumConfigurations"] = descriptor_device.bNumConfigurations;
    serializeJson(doc, Serial1);
    Serial1.println();
}

void EspUsbHost::sendEndpointDescriptors()
{
    Serial1.print("USB_sendEndpointDescriptors:");
    JsonDocument doc;
    JsonArray array = doc.to<JsonArray>();

    for (int i = 0; i < endpointCounter; ++i)
    {
        JsonObject desc = array.add<JsonObject>();
        desc["bLength"] = endpoint_descriptors[i].bLength;
        desc["bDescriptorType"] = endpoint_descriptors[i].bDescriptorType;
        desc["bEndpointAddress"] = endpoint_descriptors[i].bEndpointAddress;
        desc["endpointID"] = endpoint_descriptors[i].endpointID;
        desc["direction"] = endpoint_descriptors[i].direction;
        desc["bmAttributes"] = endpoint_descriptors[i].bmAttributes;
        desc["attributes"] = endpoint_descriptors[i].attributes;
        desc["wMaxPacketSize"] = endpoint_descriptors[i].wMaxPacketSize;
        desc["bInterval"] = endpoint_descriptors[i].bInterval;
    }
    serializeJson(doc, Serial1);
    Serial1.println();
}

void EspUsbHost::sendInterfaceDescriptors()
{
    Serial1.print("USB_sendInterfaceDescriptors:");
    JsonDocument doc;
    JsonArray array = doc.to<JsonArray>();
    for (int i = 0; i < interfaceCounter; ++i)
    {
        JsonObject desc = array.add<JsonObject>();
        desc["bLength"] = interface_descriptors[i].bLength;
        desc["bDescriptorType"] = interface_descriptors[i].bDescriptorType;
        desc["bInterfaceNumber"] = interface_descriptors[i].bInterfaceNumber;
        desc["bAlternateSetting"] = interface_descriptors[i].bAlternateSetting;
        desc["bNumEndpoints"] = interface_descriptors[i].bNumEndpoints;
        desc["bInterfaceClass"] = interface_descriptors[i].bInterfaceClass;
        desc["bInterfaceSubClass"] = interface_descriptors[i].bInterfaceSubClass;
        desc["bInterfaceProtocol"] = interface_descriptors[i].bInterfaceProtocol;
        desc["iInterface"] = interface_descriptors[i].iInterface;
    }
    serializeJson(doc, Serial1);
    Serial1.println();
}

void EspUsbHost::sendHidDescriptors()
{
    Serial1.print("USB_sendHidDescriptors:");
    JsonDocument doc;
    JsonArray array = doc.to<JsonArray>();
    for (int i = 0; i < hidDescriptorCounter; ++i)
    {
        JsonObject desc = array.add<JsonObject>();
        desc["bLength"] = hid_descriptors[i].bLength;
        desc["bDescriptorType"] = hid_descriptors[i].bDescriptorType;
        desc["bcdHID"] = hid_descriptors[i].bcdHID;
        desc["bCountryCode"] = hid_descriptors[i].bCountryCode;
        desc["bNumDescriptors"] = hid_descriptors[i].bNumDescriptors;
        desc["bReportType"] = hid_descriptors[i].bReportType;
        desc["wReportLength"] = hid_descriptors[i].wReportLength;
    }
    serializeJson(doc, Serial1);
    Serial1.println();
}

void EspUsbHost::sendIADescriptors()
{
    Serial1.print("USB_sendIADescriptors:");
    JsonDocument doc;
    doc["bLength"] = descriptor_interface_association.bLength;
    doc["bDescriptorType"] = descriptor_interface_association.bDescriptorType;
    doc["bFirstInterface"] = descriptor_interface_association.bFirstInterface;
    doc["bInterfaceCount"] = descriptor_interface_association.bInterfaceCount;
    doc["bFunctionClass"] = descriptor_interface_association.bFunctionClass;
    doc["bFunctionSubClass"] = descriptor_interface_association.bFunctionSubClass;
    doc["bFunctionProtocol"] = descriptor_interface_association.bFunctionProtocol;
    doc["iFunction"] = descriptor_interface_association.iFunction;
    serializeJson(doc, Serial1);
    Serial1.println();
}

void EspUsbHost::sendEndpointData()
{
    Serial1.print("USB_sendEndpointData:");
    JsonDocument doc;
    JsonArray array = doc.to<JsonArray>();

    for (int i = 0; i < interfaceCounter; i++)
    {
        JsonObject data = array.add<JsonObject>();
        data["bInterfaceNumber"] = endpoint_data_list[i].bInterfaceNumber;
        data["bInterfaceClass"] = endpoint_data_list[i].bInterfaceClass;
        data["bInterfaceSubClass"] = endpoint_data_list[i].bInterfaceSubClass;
        data["bInterfaceProtocol"] = endpoint_data_list[i].bInterfaceProtocol;
        data["bCountryCode"] = endpoint_data_list[i].bCountryCode;
    }
    serializeJson(doc, Serial1);
    Serial1.println();
}

void EspUsbHost::sendUnknownDescriptors()
{
    Serial1.print("USB_sendUnknownDescriptors:");
    JsonDocument doc;
    JsonArray array = doc.to<JsonArray>();
    for (int i = 0; i < unknownDescriptorCounter; ++i)
    {
        JsonObject desc = array.add<JsonObject>();
        desc["bLength"] = unknown_descriptors[i].bLength;
        desc["bDescriptorType"] = unknown_descriptors[i].bDescriptorType;
        desc["data"] = unknown_descriptors[i].data;
    }
    serializeJson(doc, Serial1);
    Serial1.println();
}

void EspUsbHost::sendDescriptorconfig()
{
    Serial1.print("USB_sendDescriptorconfig:");
    JsonDocument doc;
    doc["bLength"] = descriptor_configuration.bLength;
    doc["bDescriptorType"] = descriptor_configuration.bDescriptorType;
    doc["wTotalLength"] = descriptor_configuration.wTotalLength;
    doc["bNumInterfaces"] = descriptor_configuration.bNumInterfaces;
    doc["bConfigurationValue"] = descriptor_configuration.bConfigurationValue;
    doc["iConfiguration"] = descriptor_configuration.iConfiguration;
    doc["bmAttributes"] = descriptor_configuration.bmAttributes;
    doc["bMaxPower"] = descriptor_configuration.bMaxPower;
    serializeJson(doc, Serial1);
    Serial1.println();
}

void EspUsbHost::handleIncomingCommands(const String &command)
{
    if (command == "DEBUG_ON")
    {
        debugModeActive = true;
        EspUsbHost::current_log_level = LOG_LEVEL_PARSED;
        tx_Que("Debug mode activated.\n");

        // log(LOG_LEVEL_DEBUG, "Debug mode active. Please remove USB mouse.");  TO DO!!
    }
    else if (command == "DEBUG_OFF")
    {
        debugModeActive = false;

        log(LOG_LEVEL_DEBUG, "ESPLOG Inform: Debug mode off. Resetting system.");

        Serial1.print("USB_GOODBYE\n");

        vTaskDelay(pdMS_TO_TICKS(100));

        esp_restart();
    }
    else if (command.startsWith("DEBUG_"))
    {
        int level = command.substring(6).toInt();
        if (level >= LOG_LEVEL_OFF && level <= LOG_LEVEL_PARSED)
        {
            EspUsbHost::current_log_level = level;
            tx_Que("Log level set to %d\n", EspUsbHost::current_log_level);
        }
        else
        {
            tx_Que("Invalid log level: %d\n", level);
        }
    }
    else if (command == "READY")
    {
        if (EspUsbHost::deviceConnected)
        {
            tx_Que("USB_HELLO\n");
        }
        else
        {
            tx_Que("USB_ISNULL\n");
        }
    }
    else if (command == "USB_INIT")
    {
        deviceMouseReady = true;
    }
    else if (command == "sendDeviceInfo")
    {
        sendDeviceInfo();
    }
    else if (command == "sendDescriptorDevice")
    {
        sendDescriptorDevice();
    }
    else if (command == "sendEndpointDescriptors")
    {
        sendEndpointDescriptors();
    }
    else if (command == "sendInterfaceDescriptors")
    {
        sendInterfaceDescriptors();
    }
    else if (command == "sendHidDescriptors")
    {
        sendHidDescriptors();
    }
    else if (command == "sendIADescriptors")
    {
        sendIADescriptors();
    }
    else if (command == "sendEndpointData")
    {
        sendEndpointData();
    }
    else if (command == "sendUnknownDescriptors")
    {
        sendUnknownDescriptors();
    }
    else if (command == "sendDescriptorconfig")
    {
        sendDescriptorconfig();
    }
    else if (command == "YIELD")
    {
        enableYield = true;
    }
    else if (command == "YIELD_END")
    {
        enableYield = false;
    }
    else
    {
        tx_Que("Unknown command received: %s\n", command.c_str());
    }
}
