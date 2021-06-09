#include "otto_communication.pb.h"

#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"

#define UNKNOWN 0
#define WAITING_CONFIG 1
#define READY 2
#define RUNNING 3
#define H_BRIDGE_FAULT_1 4
#define H_BRIDGE_FAULT_2 5
#define UNKNOWN_ERROR 6

int32_t gFSMCurrentStatus = UNKNOWN;
int8_t gIndicatorLedState = LOW;

uint8_t pbStreamBuffer[128];

int32_t gPreviousElapsedTimeMs = 0;
int32_t gInducedDelay = 0;

#define CALIBRATION_ITERS 1000
#define CALIBRATION_TARGET 100//20
#define ENGINEERING_SAFETY_CONSTANT 1

#define BASELINE 0.435
/* Federica Di Lauro, [06.02.21 15:08]
   [In reply to Federica Di Lauro]
   questo è quello di otto, non ricordo cosa venisse
   fuori per l'esercizio di matlab
 **/

// Dummy encoder values
#define WHEEL_DATA_LEN 295
const float gLeftWhlFakeData[] = {0.2, 0.2, 0.2, 0.2, 0.4, 0.4, 0.4, 0.4, 0.6, 0.6, 0.6, 0.6, 0.8, 0.8, 0.8, 0.8, 1, 1, 1, 1, 1.2, 1.2, 1.2, 1.2, 1, 1, 1, 1, 0.8, 0.8, 0.8, 0.8, 0.6, 0.6, 0.6, 0.6, 0.4, 0.4, 0.4, 0.4, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.19, 0.18, 0.17, 0.16, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.14, 0.13, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.13, 0.14, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.14, 0.13, 0.13, 0.13, 0.13, 0.13, 0.14, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15};
const float gRightWhlFakeData[] = {0.2, 0.2, 0.2, 0.2, 0.4, 0.4, 0.4, 0.4, 0.6, 0.6, 0.6, 0.6, 0.8, 0.8, 0.8, 0.8, 1, 1, 1, 1, 1.2, 1.2, 1.2, 1.2, 1, 1, 1, 1, 0.8, 0.8, 0.8, 0.8, 0.6, 0.6, 0.6, 0.6, 0.4, 0.4, 0.4, 0.4, 0.2, 0.2, 0.2, 0.2, 0.19, 0.18, 0.17, 0.16, 0.15, 0.14, 0.13, 0.12, 0.11, 0.1, 0.1, 0.1, 0.1, 0.1, 0.11, 0.12, 0.13, 0.14, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.14, 0.14, 0.13, 0.13, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.13, 0.13, 0.14, 0.14, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.13, 0.12, 0.11, 0.1, 0.09, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.09, 0.1, 0.11, 0.12, 0.13, 0.14, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.14, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15};
uint32_t gFakeDataCurIdx = 0;

// Set the values to be encoded by protobuf and sent
inline void MsgSetValues(StatusMessage *msg, float left_wheel_vel, float right_wheel_vel)
{
  msg->linear_velocity = (left_wheel_vel + right_wheel_vel) / 2;
  msg->angular_velocity = (right_wheel_vel - left_wheel_vel) / BASELINE;
}

void SysCalibrationLoop(void)
{
  Serial.print("Calibrating against a target of ");
  Serial.print(CALIBRATION_TARGET);
  Serial.print("ms over ");
  Serial.print(CALIBRATION_ITERS);
  Serial.println(" passes...");
  gPreviousElapsedTimeMs = millis();
  for (int i = 0; i < CALIBRATION_ITERS; i++)
  {
    unsigned long CurrentElapsedTimeMs = millis();

    StatusMessage message = StatusMessage_init_zero;

    pb_ostream_t stream = pb_ostream_from_buffer(pbStreamBuffer, sizeof(pbStreamBuffer));

    MsgSetValues(&message, 0.0, 0.0);
    message.delta_millis = CurrentElapsedTimeMs - gPreviousElapsedTimeMs;
    message.status = gFSMCurrentStatus;

    gInducedDelay += message.delta_millis;
    gPreviousElapsedTimeMs = CurrentElapsedTimeMs;

    if (!pb_encode(&stream, StatusMessage_fields, &message))
    {
      Serial.println("Failed to encode");
      gFSMCurrentStatus = UNKNOWN_ERROR;
    }

    for (int i = 0; i < stream.bytes_written; i++)
    {
      Serial.printf("%02X", pbStreamBuffer[i]);
    }
    Serial.println();
    Serial.flush();
  }
  gInducedDelay /= CALIBRATION_ITERS;
  gInducedDelay -= CALIBRATION_TARGET;
  if (gInducedDelay > 0)
  {
    Serial.print("System cannot reliably send messages each ");
    Serial.print(CALIBRATION_TARGET);
    Serial.print("ms, avg error: ");
    gFSMCurrentStatus = UNKNOWN_ERROR;
  }
  else
  {
    Serial.print("Calibration OK, introducing a delay of ");
    gInducedDelay = abs(gInducedDelay) - ENGINEERING_SAFETY_CONSTANT;
  }
  Serial.print(gInducedDelay);
  Serial.println("ms");
  return;
}

void setup()
{
  pinMode(5, INPUT);
  pinMode(16, OUTPUT);
  Serial.begin(9600);
  delay(1000);
  //Serial.println("OTTO266 READY TO CONQUER!");
  Serial.flush();
  gFSMCurrentStatus = WAITING_CONFIG;
  //SysCalibrationLoop();
  //Serial.println("Pull D1 up to start transferring data...");
  //Serial.flush();
  // update elapsed time to avoid overshot deltat on first message
  gInducedDelay = 100;
  gPreviousElapsedTimeMs = millis();
}

void loop()
{
  delay(gInducedDelay);
  digitalWrite(16, gIndicatorLedState);
  if (digitalRead(5))
  {
    gIndicatorLedState = LOW;
    unsigned long current_tx_millis = millis();

    StatusMessage message = StatusMessage_init_zero;

    pb_ostream_t stream = pb_ostream_from_buffer(pbStreamBuffer, sizeof(pbStreamBuffer));

    MsgSetValues(&message, gLeftWhlFakeData[gFakeDataCurIdx], gRightWhlFakeData[gFakeDataCurIdx]);
    message.delta_millis = current_tx_millis - gPreviousElapsedTimeMs;
    message.status = gFSMCurrentStatus;

    gPreviousElapsedTimeMs = current_tx_millis;

    if (!pb_encode(&stream, StatusMessage_fields, &message))
    {
      Serial.println("Failed to encode");
      gFSMCurrentStatus = UNKNOWN_ERROR;
    }

    for (int i = 0; i < stream.bytes_written; i++)
    {
      //Serial.printf("%02X", pbStreamBuffer[i]);
      Serial.write(pbStreamBuffer[i]);
    }
    //Serial.println();
    Serial.flush();
    if (++gFakeDataCurIdx >= WHEEL_DATA_LEN)
      gFakeDataCurIdx = 0;
  }
  else
  {
    gIndicatorLedState = HIGH;
  }
}
