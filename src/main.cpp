#include "esp_camera.h"
#include "SPI.h"
#include "driver/rtc_io.h"
#include <ESP_Mail_Client.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <QueueArray.h>
#include <ArduinoQueue.h>
#include "SD_MMC.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "SD.h"

// Network credentials
const char *ssid = "Freedom";
const char *password = "1/2mV22@";

// Email credentials
#define emailSenderAccount "fakatrobotics@gmail.com"
#define emailSenderPassword "ebrrrnzfzrcjnbwb"
#define smtpServer "smtp.gmail.com"
#define smtpServerPort 465
#define emailSubject "ESP32-CAM Photo Captured"
#define emailRecipient "faizan.solver@gmail.com"

// Camera model configuration
#define CAMERA_MODEL_AI_THINKER
#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#else
#error "Camera model not selected"
#endif

const int touchPin = 13; // Touch sensor pin
bool touched = false;

int delayTime = 15; // Initial delay time for servo

// SMTP session object for email
SMTPSession smtp;

// Define the image queue
QueueArray<String> imageQueue;
#define MAX_QUEUE_SIZE 10

// File path for saving photos
#define FILE_PHOTO_PATH "/photo_%d.jpg"

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

const int servoPin = 12;
Servo myServo;

TaskHandle_t Task1;
TaskHandle_t Task2;

bool sendPhotoFlag = false;

// Global counter for photo filenames
int photoCounter = 0;

SemaphoreHandle_t mutex; // Mutex declaration

void smtpCallback(SMTP_Status status);
void Task1code(void *pvParameters);
void Task2code(void *pvParameters);
void capturePhotoSaveLittleFS(); //Core 1 function.
bool sendPhoto(String i); //Core 2 function.
void moveServo(int pos); //Core 1 function.

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector

  Serial.begin(115200);
  Serial.println();
  //moveServo(180);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP Address: http://");
  Serial.println(WiFi.localIP());

  // Initialize SD card
  if (!SD_MMC.begin())
  {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    return;
  }
  Serial.println("SD card initialized");

  // Initialize mutex
  mutex = xSemaphoreCreateMutex();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound())
  {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 1;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  pinMode(touchPin, INPUT);

  myServo.attach(servoPin);

  // Create tasks
  xTaskCreatePinnedToCore(
      Task1code,
      "Task1",
      10000,
      NULL,
      1,
      &Task1,
      0);

  xTaskCreatePinnedToCore(
      Task2code,
      "Task2",
      10000,
      NULL,
      1,
      &Task2,
      1);
}

void Task1code(void *pvParameters)
{
  Serial.print("Task1 running on core faizan ");
  Serial.println(xPortGetCoreID());

  bool doorLocked = true;

  while (true)
  {
    int touchState = digitalRead(touchPin);

    if (touchState == HIGH && !touched && (millis() - lastDebounceTime) > debounceDelay)
    {
      Serial.println("Sensor touched");
      touched = true;
      lastDebounceTime = millis();
      if (doorLocked)
      {
        moveServo(0);
        Serial.println("Door is unlocked !!!");
        capturePhotoSaveLittleFS();
        sendPhotoFlag = true;
        doorLocked = false;
      }
      else
      {
        moveServo(180);
        doorLocked = true;
        Serial.println("Door is locked !!!");
      }
    }
    else if (touchState == LOW && touched)
    {
      touched = false;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

bool wasQueueEmpty = false;

void Task2code(void *pvParameters)
{
  Serial.print("Task2 running on core batman ");
  Serial.println(xPortGetCoreID());

  while (true)
  {
    if (!imageQueue.isEmpty())
    {
      // Acquire mutex before accessing imageQueue
      xSemaphoreTake(mutex, portMAX_DELAY);

      String imageName = imageQueue.peek();
      Serial.print("Sending photo: ");
      Serial.println(imageName);

      // Release mutex after peeking
      xSemaphoreGive(mutex);

      if (sendPhoto(imageName))
      {
        // Acquire mutex before popping from imageQueue
        xSemaphoreTake(mutex, portMAX_DELAY);
        imageQueue.pop();
        // Release mutex after popping
        xSemaphoreGive(mutex);
      }

      wasQueueEmpty = false;
    }
    else
    {
      if (!wasQueueEmpty)
      {
        Serial.println("Queue empty");
        wasQueueEmpty = true;
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void capturePhotoSaveLittleFS()
{
  // Acquire mutex before checking and manipulating imageQueue
  xSemaphoreTake(mutex, portMAX_DELAY);

  if (imageQueue.isFull())
  {
    Serial.println("Queue is full. Cannot capture more images.");
    xSemaphoreGive(mutex);
    return;
  }

  xSemaphoreGive(mutex);

  camera_fb_t *fb = NULL;
  for (int i = 0; i < 3; i++)
  {
    fb = esp_camera_fb_get();
    esp_camera_fb_return(fb);
    fb = NULL;
  }

  fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }

  // Generate a unique filename using the photo counter
  char filename[20];
  sprintf(filename, "/photo_%d.jpg", photoCounter++); // Use photoCounter for unique filenames

  // Open file for writing
  Serial.println("test1");
  Serial.printf("Opening file: %s\n", filename);
  File file = SD_MMC.open(filename, FILE_WRITE, true);
  Serial.println("test2");
  if (!file)
  {
    Serial.println("test3");
    Serial.println("Failed to open file for writing");
    esp_camera_fb_return(fb);
    return;
  }

  // Write image data to file
  if (!file)
  {
    Serial.println("Failed to open file in writing mode");
  }
  else
  {
    Serial.println("test4");
    size_t bytesWritten = file.write(fb->buf, fb->len);
    // Acquire mutex before pushing to imageQueue
    xSemaphoreTake(mutex, portMAX_DELAY);
    imageQueue.enqueue(filename);
    // Release mutex after pushing to imageQueue
    xSemaphoreGive(mutex);
    if (bytesWritten == 0)
    {
      Serial.println("Failed to write to file");
      file.close();
      esp_camera_fb_return(fb);
      return;
    }
  }

  Serial.printf("Saved file to: %s\n", filename);
  // Close file
  file.close();
  esp_camera_fb_return(fb);
}

bool sendPhoto(String imageName)
{
  smtp.debug(1);
  smtp.callback(smtpCallback);

  Session_Config config;

  config.time.ntp_server = F("pool.ntp.org,time.nist.gov");
  config.time.gmt_offset = 0;
  config.time.day_light_offset = 1;

  config.server.host_name = smtpServer;
  config.server.port = smtpServerPort;
  config.login.email = emailSenderAccount;
  config.login.password = emailSenderPassword;
  config.login.user_domain = "";

  SMTP_Message message;
  message.sender.name = "ESP32-CAM";
  message.sender.email = emailSenderAccount;
  message.subject = emailSubject;
  message.addRecipient("Recipient", emailRecipient);

  String htmlMsg = "<div style=\"color:#2f4468;\"><h1>ESP32-CAM Photo</h1><p>This is an image taken with the ESP32-CAM.</p></div>";
  message.html.content = htmlMsg.c_str();
  message.html.charSet = "utf-8";
  message.html.transfer_encoding = Content_Transfer_Encoding::enc_qp;

  message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_normal;
  message.response.notify = esp_mail_smtp_notify_success | esp_mail_smtp_notify_failure | esp_mail_smtp_notify_delay;

  SMTP_Attachment att;

  att.descr.filename = imageName;
  att.descr.mime = "image/jpeg";
  char fullPath[50]; // Adjust the size according to your needs
  // Next 2 lines was solution if it worked! else uncomment 3rd line.
  sprintf(fullPath, FILE_PHOTO_PATH, imageName.c_str());
  att.file.path = fullPath;
  // att.file.path = FILE_PHOTO_PATH + imageName;
  att.file.storage_type = esp_mail_file_storage_type_flash;
  att.descr.transfer_encoding = Content_Transfer_Encoding::enc_base64;

  message.addAttachment(att);

  if (!smtp.connect(&config))
    return false;

  if (!MailClient.sendMail(&smtp, &message, true))
  {
    Serial.print("Error sending Email, ");
    Serial.println(smtp.errorReason());
    return false;
  }

  return true;
}

void smtpCallback(SMTP_Status status)
{
  Serial.println(status.info());

  if (status.success())
  {
    Serial.println("----------------");
    Serial.printf("Message sent success: %d\n", status.completedCount());
    Serial.printf("Message sent failed: %d\n", status.failedCount());
    Serial.println("----------------\n");
    struct tm dt;

    for (size_t i = 0; i < smtp.sendingResult.size(); i++)
    {
      SMTP_Result result = smtp.sendingResult.getItem(i);
      time_t ts = (time_t)result.timestamp;
      localtime_r(&ts, &dt);

      ESP_MAIL_PRINTF("Message No: %d\n", i + 1);
      ESP_MAIL_PRINTF("Status: %s\n", result.completed ? "success" : "failed");
      ESP_MAIL_PRINTF("Date/Time: %d/%d/%d %d:%d:%d\n", dt.tm_year + 1900, dt.tm_mon + 1, dt.tm_mday, dt.tm_hour, dt.tm_min, dt.tm_sec);
      ESP_MAIL_PRINTF("Recipient: %s\n", result.recipients.c_str());
      ESP_MAIL_PRINTF("Subject: %s\n", result.subject.c_str());
    }
    Serial.println("----------------\n");
    smtp.sendingResult.clear();
  }
}

void moveServo(int pos)
{
  if (pos == 0)
  {
    for (int i = 0; i <= 180; i++)
    {
      myServo.write(i);
      vTaskDelay(delayTime / portTICK_PERIOD_MS);
    }
  }
  else if (pos == 180)
  {
    for (int i = 180; i >= 0; i--)
    {
      myServo.write(i);
      vTaskDelay(delayTime / portTICK_PERIOD_MS);
    }
  }
}

void loop()
{
  // Empty. Tasks are handled by FreeRTOS
}
