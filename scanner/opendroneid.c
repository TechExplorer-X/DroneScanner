/* SPDX-License-Identifier: Apache-2.0 */
/* Open Drone ID C Library */

#include "opendroneid.h"
#include <math.h>
#include <stdio.h>
#define ENABLE_DEBUG 1

const float SPEED_DIV[2] = {0.25f, 0.75f};
const float VSPEED_DIV = 0.5f;
const int32_t LATLON_MULT = 10000000;
const float ALT_DIV = 0.5f;
const int ALT_ADDER = 1000;

static char *safe_dec_copyfill(char *dstStr, const char *srcStr, int dstSize);
static int intRangeMax(int64_t inValue, int startRange, int endRange);
static int intInRange(int inValue, int startRange, int endRange);


void odid_initBasicIDData(ODID_BasicID_data *data)
{
    if (!data)
        return;
    memset(data, 0, sizeof(ODID_BasicID_data));
}


void odid_initLocationData(ODID_Location_data *data)
{
    if (!data)
        return;
    memset(data, 0, sizeof(ODID_Location_data));
    data->Direction = INV_DIR;
    data->SpeedHorizontal = INV_SPEED_H;
    data->SpeedVertical = INV_SPEED_V;
    data->AltitudeBaro = INV_ALT;
    data->AltitudeGeo = INV_ALT;
    data->Height = INV_ALT;
}


void odid_initAuthData(ODID_Auth_data *data)
{
    if (!data)
        return;
    memset(data, 0, sizeof(ODID_Auth_data));
}


void odid_initSelfIDData(ODID_SelfID_data *data)
{
    if (!data)
        return;
    memset(data, 0, sizeof(ODID_SelfID_data));
}


void odid_initSystemData(ODID_System_data *data)
{
    if (!data)
        return;
    memset(data, 0, sizeof(ODID_System_data));
    data->AreaCount = 1;
    data->AreaCeiling = INV_ALT;
    data->AreaFloor = INV_ALT;
    data->OperatorAltitudeGeo = INV_ALT;
}


void odid_initOperatorIDData(ODID_OperatorID_data *data)
{
    if (!data)
        return;
    memset(data, 0, sizeof(ODID_OperatorID_data));
}


void odid_initMessagePackData(ODID_MessagePack_data *data)
{
    if (!data)
        return;
    memset(data, 0, sizeof(ODID_MessagePack_data));
    data->SingleMessageSize = ODID_MESSAGE_SIZE;
}


void odid_initUasData(ODID_UAS_Data *data)
{
    if (!data)
        return;
    for (int i = 0; i < ODID_BASIC_ID_MAX_MESSAGES; i++) {
        data->BasicIDValid[i] = 0;
        odid_initBasicIDData(&data->BasicID[i]);
    }
    data->LocationValid = 0;
    odid_initLocationData(&data->Location);
    for (int i = 0; i < ODID_AUTH_MAX_PAGES; i++) {
        data->AuthValid[i] = 0;
        odid_initAuthData(&data->Auth[i]);
    }
    data->SelfIDValid = 0;
    odid_initSelfIDData(&data->SelfID);
    data->SystemValid = 0;
    odid_initSystemData(&data->System);
    data->OperatorIDValid = 0;
    odid_initOperatorIDData(&data->OperatorID);
}


static uint8_t encodeDirection(float Direction, uint8_t *EWDirection)
{
    unsigned int direction_int = (unsigned int) roundf(Direction);
    if (direction_int < 180) {
        *EWDirection = 0;
    } else {
        *EWDirection = 1;
        direction_int -= 180;
    }
    return (uint8_t) intRangeMax(direction_int, 0, UINT8_MAX);
}


static uint8_t encodeSpeedHorizontal(float Speed_data, uint8_t *mult)
{
    if (Speed_data <= UINT8_MAX * SPEED_DIV[0]) {
        *mult = 0;
        return (uint8_t) (Speed_data / SPEED_DIV[0]);
    } else {
        *mult = 1;
        int big_value = (int) ((Speed_data - (UINT8_MAX * SPEED_DIV[0])) / SPEED_DIV[1]);
        return (uint8_t) intRangeMax(big_value, 0, UINT8_MAX);
    }
}


static int8_t encodeSpeedVertical(float SpeedVertical_data)
{
    int encValue = (int) (SpeedVertical_data / VSPEED_DIV);
    return (int8_t) intRangeMax(encValue, INT8_MIN, INT8_MAX);
}


static int32_t encodeLatLon(double LatLon_data)
{
    return (int32_t) intRangeMax((int64_t) (LatLon_data * LATLON_MULT), -180 * LATLON_MULT, 180 * LATLON_MULT);
}


static uint16_t encodeAltitude(float Alt_data)
{
    return (uint16_t) intRangeMax( (int) ((Alt_data + (float) ALT_ADDER) / ALT_DIV), 0, UINT16_MAX);
}


static uint16_t encodeTimeStamp(float Seconds_data)
{
    if (Seconds_data == INV_TIMESTAMP)
        return INV_TIMESTAMP;
    else
        return (uint16_t) intRangeMax((int64_t) roundf(Seconds_data*10), 0, MAX_TIMESTAMP * 10);
}


static uint8_t encodeAreaRadius(uint16_t Radius)
{
    return (uint8_t) intRangeMax(Radius / 10, 0, 255);
}


int encodeBasicIDMessage(ODID_BasicID_encoded *outEncoded, ODID_BasicID_data *inData)
{
    if (!outEncoded || !inData ||
        !intInRange(inData->IDType, 0, 15) ||
        !intInRange(inData->UAType, 0, 15))
        return ODID_FAIL;

    outEncoded->MessageType = ODID_MESSAGETYPE_BASIC_ID;
    outEncoded->ProtoVersion = ODID_PROTOCOL_VERSION;
    outEncoded->IDType = inData->IDType;
    outEncoded->UAType = inData->UAType;
    strncpy(outEncoded->UASID, inData->UASID, sizeof(outEncoded->UASID));
    memset(outEncoded->Reserved, 0, sizeof(outEncoded->Reserved));
    return ODID_SUCCESS;
}


int encodeLocationMessage(ODID_Location_encoded *outEncoded, ODID_Location_data *inData)
{
    uint8_t bitflag;
    if (!outEncoded || !inData ||
        !intInRange(inData->Status, 0, 15) ||
        !intInRange(inData->HeightType, 0, 1) ||
        !intInRange(inData->HorizAccuracy, 0, 15) ||
        !intInRange(inData->VertAccuracy, 0, 15) ||
        !intInRange(inData->BaroAccuracy, 0, 15) ||
        !intInRange(inData->SpeedAccuracy, 0, 15) ||
        !intInRange(inData->TSAccuracy, 0, 15))
        return ODID_FAIL;

    if (inData->Direction < MIN_DIR || inData->Direction > INV_DIR ||
        (inData->Direction > MAX_DIR && inData->Direction < INV_DIR))
        return ODID_FAIL;

    if (inData->SpeedHorizontal < MIN_SPEED_H || inData->SpeedHorizontal > INV_SPEED_H ||
        (inData->SpeedHorizontal > MAX_SPEED_H && inData->SpeedHorizontal < INV_SPEED_H))
        return ODID_FAIL;

    if (inData->SpeedVertical < MIN_SPEED_V || inData->SpeedVertical > INV_SPEED_V ||
        (inData->SpeedVertical > MAX_SPEED_V && inData->SpeedVertical < INV_SPEED_V))
        return ODID_FAIL;

    if (inData->Latitude < MIN_LAT || inData->Latitude > MAX_LAT ||
        inData->Longitude < MIN_LON || inData->Longitude > MAX_LON)
        return ODID_FAIL;

    if (inData->AltitudeBaro < MIN_ALT || inData->AltitudeBaro > MAX_ALT ||
        inData->AltitudeGeo < MIN_ALT || inData->AltitudeGeo > MAX_ALT ||
        inData->Height < MIN_ALT || inData->Height > MAX_ALT)
        return ODID_FAIL;

    if (inData->TimeStamp < 0 ||
        (inData->TimeStamp > MAX_TIMESTAMP && inData->TimeStamp != INV_TIMESTAMP))
        return ODID_FAIL;

    outEncoded->MessageType = ODID_MESSAGETYPE_LOCATION;
    outEncoded->ProtoVersion = ODID_PROTOCOL_VERSION;
    outEncoded->Status = inData->Status;
    outEncoded->Reserved = 0;
    outEncoded->Direction = encodeDirection(inData->Direction, &bitflag);
    outEncoded->EWDirection = bitflag;
    outEncoded->SpeedHorizontal = encodeSpeedHorizontal(inData->SpeedHorizontal, &bitflag);
    outEncoded->SpeedMult = bitflag;
    outEncoded->SpeedVertical = encodeSpeedVertical(inData->SpeedVertical);
    outEncoded->Latitude = encodeLatLon(inData->Latitude);
    outEncoded->Longitude = encodeLatLon(inData->Longitude);
    outEncoded->AltitudeBaro = encodeAltitude(inData->AltitudeBaro);
    outEncoded->AltitudeGeo = encodeAltitude(inData->AltitudeGeo);
    outEncoded->HeightType = inData->HeightType;
    outEncoded->Height = encodeAltitude(inData->Height);
    outEncoded->HorizAccuracy = inData->HorizAccuracy;
    outEncoded->VertAccuracy = inData->VertAccuracy;
    outEncoded->BaroAccuracy = inData->BaroAccuracy;
    outEncoded->SpeedAccuracy = inData->SpeedAccuracy;
    outEncoded->TSAccuracy = inData->TSAccuracy;
    outEncoded->Reserved2 = 0;
    outEncoded->TimeStamp = encodeTimeStamp(inData->TimeStamp);
    outEncoded->Reserved3 = 0;
    return ODID_SUCCESS;
}


int encodeAuthMessage(ODID_Auth_encoded *outEncoded, ODID_Auth_data *inData)
{
    if (!outEncoded || !inData || !intInRange(inData->AuthType, 0, 15))
        return ODID_FAIL;

    if (inData->DataPage >= ODID_AUTH_MAX_PAGES)
        return ODID_FAIL;

    if (inData->DataPage == 0) {
        if (inData->LastPageIndex >= ODID_AUTH_MAX_PAGES)
            return ODID_FAIL;

#if (MAX_AUTH_LENGTH < UINT8_MAX)
        if (inData->Length > MAX_AUTH_LENGTH)
            return ODID_FAIL;
#endif

        int len = ODID_AUTH_PAGE_ZERO_DATA_SIZE +
                  inData->LastPageIndex * ODID_AUTH_PAGE_NONZERO_DATA_SIZE;
        if (len < inData->Length)
            return ODID_FAIL;
    }

    outEncoded->page_zero.MessageType = ODID_MESSAGETYPE_AUTH;
    outEncoded->page_zero.ProtoVersion = ODID_PROTOCOL_VERSION;
    outEncoded->page_zero.AuthType = inData->AuthType;
    outEncoded->page_zero.DataPage = inData->DataPage;

    if (inData->DataPage == 0) {
        outEncoded->page_zero.LastPageIndex = inData->LastPageIndex;
        outEncoded->page_zero.Length = inData->Length;
        outEncoded->page_zero.Timestamp = inData->Timestamp;
        memcpy(outEncoded->page_zero.AuthData, inData->AuthData,
               sizeof(outEncoded->page_zero.AuthData));
    } else {
        memcpy(outEncoded->page_non_zero.AuthData, inData->AuthData,
               sizeof(outEncoded->page_non_zero.AuthData));
    }
    return ODID_SUCCESS;
}


int encodeSelfIDMessage(ODID_SelfID_encoded *outEncoded, ODID_SelfID_data *inData)
{
    if (!outEncoded || !inData || !intInRange(inData->DescType, 0, 255))
        return ODID_FAIL;

    outEncoded->MessageType = ODID_MESSAGETYPE_SELF_ID;
    outEncoded->ProtoVersion = ODID_PROTOCOL_VERSION;
    outEncoded->DescType = inData->DescType;
    strncpy(outEncoded->Desc, inData->Desc, sizeof(outEncoded->Desc));
    return ODID_SUCCESS;
}


int encodeSystemMessage(ODID_System_encoded *outEncoded, ODID_System_data *inData)
{
    if (!outEncoded || !inData ||
        !intInRange(inData->OperatorLocationType, 0, 3) ||
        !intInRange(inData->ClassificationType, 0, 7) ||
        !intInRange(inData->CategoryEU, 0, 15) ||
        !intInRange(inData->ClassEU, 0, 15))
        return ODID_FAIL;

    if (inData->OperatorLatitude < MIN_LAT || inData->OperatorLatitude > MAX_LAT ||
        inData->OperatorLongitude < MIN_LON || inData->OperatorLongitude > MAX_LON)
        return ODID_FAIL;

    if (inData->AreaRadius > MAX_AREA_RADIUS)
        return ODID_FAIL;

    if (inData->AreaCeiling < MIN_ALT || inData->AreaCeiling > MAX_ALT ||
        inData->AreaFloor < MIN_ALT || inData->AreaFloor > MAX_ALT ||
        inData->OperatorAltitudeGeo < MIN_ALT || inData->OperatorAltitudeGeo > MAX_ALT)
        return ODID_FAIL;

    outEncoded->MessageType = ODID_MESSAGETYPE_SYSTEM;
    outEncoded->ProtoVersion = ODID_PROTOCOL_VERSION;
    outEncoded->Reserved = 0;
    outEncoded->OperatorLocationType = inData->OperatorLocationType;
    outEncoded->ClassificationType = inData->ClassificationType;
    outEncoded->OperatorLatitude = encodeLatLon(inData->OperatorLatitude);
    outEncoded->OperatorLongitude = encodeLatLon(inData->OperatorLongitude);
    outEncoded->AreaCount = inData->AreaCount;
    outEncoded->AreaRadius = encodeAreaRadius(inData->AreaRadius);
    outEncoded->AreaCeiling = encodeAltitude(inData->AreaCeiling);
    outEncoded->AreaFloor = encodeAltitude(inData->AreaFloor);
    outEncoded->CategoryEU = inData->CategoryEU;
    outEncoded->ClassEU = inData->ClassEU;
    outEncoded->OperatorAltitudeGeo = encodeAltitude(inData->OperatorAltitudeGeo);
    outEncoded->Timestamp = inData->Timestamp;
    outEncoded->Reserved2 = 0;
    return ODID_SUCCESS;
}


int encodeOperatorIDMessage(ODID_OperatorID_encoded *outEncoded, ODID_OperatorID_data *inData)
{
    if (!outEncoded || !inData || !intInRange(inData->OperatorIdType, 0, 255))
        return ODID_FAIL;

    outEncoded->MessageType = ODID_MESSAGETYPE_OPERATOR_ID;
    outEncoded->ProtoVersion = ODID_PROTOCOL_VERSION;
    outEncoded->OperatorIdType = inData->OperatorIdType;
    strncpy(outEncoded->OperatorId, inData->OperatorId, sizeof(outEncoded->OperatorId));
    memset(outEncoded->Reserved, 0, sizeof(outEncoded->Reserved));
    return ODID_SUCCESS;
}


static int checkPackContent(ODID_Message_encoded *msgs, int amount)
{
    if (amount <= 0 || amount > ODID_PACK_MAX_MESSAGES)
        return ODID_FAIL;

    int numMessages[6] = { 0 }; // Counters for relevant parts of ODID_messagetype_t
    for (int i = 0; i < amount; i++) {
        uint8_t MessageType = decodeMessageType(msgs[i].rawData[0]);


        if (MessageType <= ODID_MESSAGETYPE_OPERATOR_ID)
            numMessages[MessageType]++;
        else
            return ODID_FAIL;
    }

    if (numMessages[ODID_MESSAGETYPE_BASIC_ID] > ODID_BASIC_ID_MAX_MESSAGES ||
        numMessages[ODID_MESSAGETYPE_LOCATION] > 1 ||
        numMessages[ODID_MESSAGETYPE_AUTH] > ODID_AUTH_MAX_PAGES ||
        numMessages[ODID_MESSAGETYPE_SELF_ID] > 1 ||
        numMessages[ODID_MESSAGETYPE_SYSTEM] > 1 ||
        numMessages[ODID_MESSAGETYPE_OPERATOR_ID] > 1)
        return ODID_FAIL;

    return ODID_SUCCESS;
}


int encodeMessagePack(ODID_MessagePack_encoded *outEncoded, ODID_MessagePack_data *inData)
{
    if (!outEncoded || !inData || inData->SingleMessageSize != ODID_MESSAGE_SIZE)
        return ODID_FAIL;

    if (checkPackContent(inData->Messages, inData->MsgPackSize) != ODID_SUCCESS)
        return ODID_FAIL;

    outEncoded->MessageType = ODID_MESSAGETYPE_PACKED;
    outEncoded->ProtoVersion = ODID_PROTOCOL_VERSION;

    outEncoded->SingleMessageSize = inData->SingleMessageSize;
    outEncoded->MsgPackSize = inData->MsgPackSize;

    for (int i = 0; i < inData->MsgPackSize; i++)
        memcpy(&outEncoded->Messages[i], &inData->Messages[i], ODID_MESSAGE_SIZE);

    return ODID_SUCCESS;
}


static float decodeDirection(uint8_t Direction_enc, uint8_t EWDirection)
{
    if (EWDirection)
        return (float) Direction_enc + 180;
    else
        return (float) Direction_enc;
}


static float decodeSpeedHorizontal(uint8_t Speed_enc, uint8_t mult)
{
    if (mult)
        return ((float) Speed_enc * SPEED_DIV[1]) + (UINT8_MAX * SPEED_DIV[0]);
    else
        return (float) Speed_enc * SPEED_DIV[0];
}


static float decodeSpeedVertical(int8_t SpeedVertical_enc)
{
    return (float) SpeedVertical_enc * VSPEED_DIV;
}


static double decodeLatLon(int32_t LatLon_enc)
{
    return (double) LatLon_enc / LATLON_MULT;
}


static float decodeAltitude(uint16_t Alt_enc)
{
    return (float) Alt_enc * ALT_DIV - (float) ALT_ADDER;
}


static float decodeTimeStamp(uint16_t Seconds_enc)
{
    if (Seconds_enc == INV_TIMESTAMP)
        return INV_TIMESTAMP;
    else
        return (float) Seconds_enc / 10;
}


static uint16_t decodeAreaRadius(uint8_t Radius_enc)
{
    return (uint16_t) ((int) Radius_enc * 10);
}


int getBasicIDType(ODID_BasicID_encoded *inEncoded, enum ODID_idtype *idType)
{
    if (!inEncoded || !idType || inEncoded->MessageType != ODID_MESSAGETYPE_BASIC_ID)
        return ODID_FAIL;

    *idType = (enum ODID_idtype) inEncoded->IDType;
    return ODID_SUCCESS;
}


int decodeBasicIDMessage(ODID_BasicID_data *outData, ODID_BasicID_encoded *inEncoded)
{
    if (!outData || !inEncoded ||
        inEncoded->MessageType != ODID_MESSAGETYPE_BASIC_ID ||
        !intInRange(inEncoded->IDType, 0, 15) ||
        !intInRange(inEncoded->UAType, 0, 15))
        return ODID_FAIL;

    outData->IDType = (ODID_idtype_t) inEncoded->IDType;
    outData->UAType = (ODID_uatype_t) inEncoded->UAType;
    safe_dec_copyfill(outData->UASID, inEncoded->UASID, sizeof(outData->UASID));
    return ODID_SUCCESS;
}


int decodeLocationMessage(ODID_Location_data *outData, ODID_Location_encoded *inEncoded)
{
    if (!outData || !inEncoded ||
        inEncoded->MessageType != ODID_MESSAGETYPE_LOCATION ||
        !intInRange(inEncoded->Status, 0, 15))
        return ODID_FAIL;

    outData->Status = (ODID_status_t) inEncoded->Status;
    outData->Direction = decodeDirection(inEncoded->Direction, inEncoded-> EWDirection);
    outData->SpeedHorizontal = decodeSpeedHorizontal(inEncoded->SpeedHorizontal, inEncoded->SpeedMult);
    outData->SpeedVertical = decodeSpeedVertical(inEncoded->SpeedVertical);
    outData->Latitude = decodeLatLon(inEncoded->Latitude);
    outData->Longitude = decodeLatLon(inEncoded->Longitude);
    outData->AltitudeBaro = decodeAltitude(inEncoded->AltitudeBaro);
    outData->AltitudeGeo = decodeAltitude(inEncoded->AltitudeGeo);
    outData->HeightType = (ODID_Height_reference_t) inEncoded->HeightType;
    outData->Height = decodeAltitude(inEncoded->Height);
    outData->HorizAccuracy = (ODID_Horizontal_accuracy_t) inEncoded->HorizAccuracy;
    outData->VertAccuracy = (ODID_Vertical_accuracy_t) inEncoded->VertAccuracy;
    outData->BaroAccuracy = (ODID_Vertical_accuracy_t) inEncoded->BaroAccuracy;
    outData->SpeedAccuracy = (ODID_Speed_accuracy_t) inEncoded->SpeedAccuracy;
    outData->TSAccuracy = (ODID_Timestamp_accuracy_t) inEncoded->TSAccuracy;
    outData->TimeStamp = decodeTimeStamp(inEncoded->TimeStamp);
    return ODID_SUCCESS;
}


int getAuthPageNum(ODID_Auth_encoded *inEncoded, int *pageNum)
{
    if (!inEncoded || !pageNum ||
        inEncoded->page_zero.MessageType != ODID_MESSAGETYPE_AUTH ||
        !intInRange(inEncoded->page_zero.AuthType, 0, 15) ||
        !intInRange(inEncoded->page_zero.DataPage, 0, ODID_AUTH_MAX_PAGES - 1))
        return ODID_FAIL;

    *pageNum = inEncoded->page_zero.DataPage;
    return ODID_SUCCESS;
}


int decodeAuthMessage(ODID_Auth_data *outData, ODID_Auth_encoded *inEncoded)
{
    if (!outData || !inEncoded ||
        inEncoded->page_zero.MessageType != ODID_MESSAGETYPE_AUTH ||
        !intInRange(inEncoded->page_zero.AuthType, 0, 15) ||
        !intInRange(inEncoded->page_zero.DataPage, 0, ODID_AUTH_MAX_PAGES - 1))
        return ODID_FAIL;

    if (inEncoded->page_zero.DataPage == 0) {
        if (inEncoded->page_zero.LastPageIndex >= ODID_AUTH_MAX_PAGES)
            return ODID_FAIL;

#if (MAX_AUTH_LENGTH < UINT8_MAX)
        if (inEncoded->page_zero.Length > MAX_AUTH_LENGTH)
            return ODID_FAIL;
#endif

        int len = ODID_AUTH_PAGE_ZERO_DATA_SIZE +
                  inEncoded->page_zero.LastPageIndex * ODID_AUTH_PAGE_NONZERO_DATA_SIZE;
        if (len < inEncoded->page_zero.Length)
            return ODID_FAIL;
    }

    outData->AuthType = (ODID_authtype_t) inEncoded->page_zero.AuthType;
    outData->DataPage = inEncoded->page_zero.DataPage;
    if (inEncoded->page_zero.DataPage == 0) {
        outData->LastPageIndex = inEncoded->page_zero.LastPageIndex;
        outData->Length = inEncoded->page_zero.Length;
        outData->Timestamp = inEncoded->page_zero.Timestamp;
        memset(outData->AuthData, 0, sizeof(outData->AuthData));
        memcpy(outData->AuthData, inEncoded->page_zero.AuthData,
               ODID_AUTH_PAGE_ZERO_DATA_SIZE);
    } else {
        memset(outData->AuthData, 0, sizeof(outData->AuthData));
        memcpy(outData->AuthData, inEncoded->page_non_zero.AuthData,
               ODID_AUTH_PAGE_NONZERO_DATA_SIZE);
    }

    return ODID_SUCCESS;
}


int decodeSelfIDMessage(ODID_SelfID_data *outData, ODID_SelfID_encoded *inEncoded)
{
    if (!outData || !inEncoded ||
        inEncoded->MessageType != ODID_MESSAGETYPE_SELF_ID)
        return ODID_FAIL;

    outData->DescType = (ODID_desctype_t) inEncoded->DescType;
    safe_dec_copyfill(outData->Desc, inEncoded->Desc, sizeof(outData->Desc));
    return ODID_SUCCESS;
}


int decodeSystemMessage(ODID_System_data *outData, ODID_System_encoded *inEncoded)
{
    if (!outData || !inEncoded ||
        inEncoded->MessageType != ODID_MESSAGETYPE_SYSTEM)
        return ODID_FAIL;

    outData->OperatorLocationType =
        (ODID_operator_location_type_t) inEncoded->OperatorLocationType;
    outData->ClassificationType =
        (ODID_classification_type_t) inEncoded->ClassificationType;
    outData->OperatorLatitude = decodeLatLon(inEncoded->OperatorLatitude);
    outData->OperatorLongitude = decodeLatLon(inEncoded->OperatorLongitude);
    outData->AreaCount = inEncoded->AreaCount;
    outData->AreaRadius = decodeAreaRadius(inEncoded->AreaRadius);
    outData->AreaCeiling = decodeAltitude(inEncoded->AreaCeiling);
    outData->AreaFloor = decodeAltitude(inEncoded->AreaFloor);
    outData->CategoryEU = (ODID_category_EU_t) inEncoded->CategoryEU;
    outData->ClassEU = (ODID_class_EU_t) inEncoded->ClassEU;
    outData->OperatorAltitudeGeo = decodeAltitude(inEncoded->OperatorAltitudeGeo);
    outData->Timestamp = inEncoded->Timestamp;
    return ODID_SUCCESS;
}


int decodeOperatorIDMessage(ODID_OperatorID_data *outData, ODID_OperatorID_encoded *inEncoded)
{
    if (!outData || !inEncoded ||
        inEncoded->MessageType != ODID_MESSAGETYPE_OPERATOR_ID)
        return ODID_FAIL;

    outData->OperatorIdType = (ODID_operatorIdType_t) inEncoded->OperatorIdType;
    safe_dec_copyfill(outData->OperatorId, inEncoded->OperatorId, sizeof(outData->OperatorId));
    return ODID_SUCCESS;
}


int decodeMessagePack(ODID_UAS_Data *uasData, ODID_MessagePack_encoded *pack)
{
    if (!uasData || !pack || pack->MessageType != ODID_MESSAGETYPE_PACKED)
        return ODID_FAIL;

    if (pack->SingleMessageSize != ODID_MESSAGE_SIZE)
        return ODID_FAIL;

    if (checkPackContent(pack->Messages, pack->MsgPackSize) != ODID_SUCCESS)
        return ODID_FAIL;

    for (int i = 0; i < pack->MsgPackSize; i++) {
        decodeOpenDroneID(uasData, pack->Messages[i].rawData);
    }
    return ODID_SUCCESS;
}


ODID_messagetype_t decodeMessageType(uint8_t byte)
{
    switch (byte >> 4)
    {
    case ODID_MESSAGETYPE_BASIC_ID:
        return ODID_MESSAGETYPE_BASIC_ID;
    case ODID_MESSAGETYPE_LOCATION:
        return ODID_MESSAGETYPE_LOCATION;
    case ODID_MESSAGETYPE_AUTH:
        return ODID_MESSAGETYPE_AUTH;
    case ODID_MESSAGETYPE_SELF_ID:
        return ODID_MESSAGETYPE_SELF_ID;
    case ODID_MESSAGETYPE_SYSTEM:
        return ODID_MESSAGETYPE_SYSTEM;
    case ODID_MESSAGETYPE_OPERATOR_ID:
        return ODID_MESSAGETYPE_OPERATOR_ID;
    case ODID_MESSAGETYPE_PACKED:
        return ODID_MESSAGETYPE_PACKED;
    default:
        return ODID_MESSAGETYPE_INVALID;
    }
}


ODID_messagetype_t decodeOpenDroneID(ODID_UAS_Data *uasData, uint8_t *msgData)
{
    if (!uasData || !msgData)
        return ODID_MESSAGETYPE_INVALID;

    switch (decodeMessageType(msgData[0]))
    {
    case ODID_MESSAGETYPE_BASIC_ID: {
        ODID_BasicID_encoded *basicId = (ODID_BasicID_encoded *) msgData;
        enum ODID_idtype idType;
        if (getBasicIDType(basicId, &idType) == ODID_SUCCESS) {

            for (int i = 0; i < ODID_BASIC_ID_MAX_MESSAGES; i++) {
                enum ODID_idtype storedType = uasData->BasicID[i].IDType;
                if (storedType == ODID_IDTYPE_NONE || storedType == idType) {
                    if (decodeBasicIDMessage(&uasData->BasicID[i], basicId) == ODID_SUCCESS) {
                        uasData->BasicIDValid[i] = 1;
                        return ODID_MESSAGETYPE_BASIC_ID;
                    }
                }
            }
        }
        break;
    }
    case ODID_MESSAGETYPE_LOCATION: {
        ODID_Location_encoded *location = (ODID_Location_encoded *) msgData;
        if (decodeLocationMessage(&uasData->Location, location) == ODID_SUCCESS) {
            uasData->LocationValid = 1;
            return ODID_MESSAGETYPE_LOCATION;
        }
        break;
    }
    case ODID_MESSAGETYPE_AUTH: {
        ODID_Auth_encoded *auth = (ODID_Auth_encoded *) msgData;
        int pageNum;
        if (getAuthPageNum(auth, &pageNum) == ODID_SUCCESS) {
            ODID_Auth_data *authData = &uasData->Auth[pageNum];
            if (decodeAuthMessage(authData, auth) == ODID_SUCCESS) {
                uasData->AuthValid[pageNum] = 1;
                return ODID_MESSAGETYPE_AUTH;
            }
        }
        break;
    }
    case ODID_MESSAGETYPE_SELF_ID: {
        ODID_SelfID_encoded *selfId = (ODID_SelfID_encoded *) msgData;
        if (decodeSelfIDMessage(&uasData->SelfID, selfId) == ODID_SUCCESS) {
            uasData->SelfIDValid = 1;
            return ODID_MESSAGETYPE_SELF_ID;
        }
        break;
    }
    case ODID_MESSAGETYPE_SYSTEM: {
        ODID_System_encoded *system = (ODID_System_encoded *) msgData;
        if (decodeSystemMessage(&uasData->System, system) == ODID_SUCCESS) {
            uasData->SystemValid = 1;
            return ODID_MESSAGETYPE_SYSTEM;
        }
        break;
    }
    case ODID_MESSAGETYPE_OPERATOR_ID: {
        ODID_OperatorID_encoded *operatorId = (ODID_OperatorID_encoded *) msgData;
        if (decodeOperatorIDMessage(&uasData->OperatorID, operatorId) == ODID_SUCCESS) {
            uasData->OperatorIDValid = 1;
            return ODID_MESSAGETYPE_OPERATOR_ID;
        }
        break;
    }
    case ODID_MESSAGETYPE_PACKED: {
        ODID_MessagePack_encoded *pack = (ODID_MessagePack_encoded *) msgData;
        if (decodeMessagePack(uasData, pack) == ODID_SUCCESS)
            return ODID_MESSAGETYPE_PACKED;
        break;
    }
    default:
        break;
    }

    return ODID_MESSAGETYPE_INVALID;
}


static char *safe_dec_copyfill(char *dstStr, const char *srcStr, int dstSize)
{
    memset(dstStr, 0, dstSize);  // fills destination with nulls
    strncpy(dstStr, srcStr, dstSize-1); // copy only up to dst size-1 (no overruns)
    return dstStr;
}


static int intRangeMax(int64_t inValue, int startRange, int endRange) {
    if ( inValue < startRange ) {
        return startRange;
    } else if (inValue > endRange) {
        return endRange;
    } else {
        return (int) inValue;
    }
}


static int intInRange(int inValue, int startRange, int endRange)
{
    if (inValue < startRange || inValue > endRange) {
        return 0;
    } else {
        return 1;
    }
}


ODID_Horizontal_accuracy_t createEnumHorizontalAccuracy(float Accuracy)
{
    if (Accuracy >= 18520)
        return ODID_HOR_ACC_UNKNOWN;
    else if (Accuracy >= 7408)
        return ODID_HOR_ACC_10NM;
    else if (Accuracy >= 3704)
        return ODID_HOR_ACC_4NM;
    else if (Accuracy >= 1852)
        return ODID_HOR_ACC_2NM;
    else if (Accuracy >= 926)
        return ODID_HOR_ACC_1NM;
    else if (Accuracy >= 555.6f)
        return ODID_HOR_ACC_0_5NM;
    else if (Accuracy >= 185.2f)
        return ODID_HOR_ACC_0_3NM;
    else if (Accuracy >= 92.6f)
        return ODID_HOR_ACC_0_1NM;
    else if (Accuracy >= 30)
        return ODID_HOR_ACC_0_05NM;
    else if (Accuracy >= 10)
        return ODID_HOR_ACC_30_METER;
    else if (Accuracy >= 3)
        return ODID_HOR_ACC_10_METER;
    else if (Accuracy >= 1)
        return ODID_HOR_ACC_3_METER;
    else if (Accuracy > 0)
        return ODID_HOR_ACC_1_METER;
    else
        return ODID_HOR_ACC_UNKNOWN;
}


ODID_Vertical_accuracy_t createEnumVerticalAccuracy(float Accuracy)
{
    if (Accuracy >= 150)
        return ODID_VER_ACC_UNKNOWN;
    else if (Accuracy >= 45)
        return ODID_VER_ACC_150_METER;
    else if (Accuracy >= 25)
        return ODID_VER_ACC_45_METER;
    else if (Accuracy >= 10)
        return ODID_VER_ACC_25_METER;
    else if (Accuracy >= 3)
        return ODID_VER_ACC_10_METER;
    else if (Accuracy >= 1)
        return ODID_VER_ACC_3_METER;
    else if (Accuracy > 0)
        return ODID_VER_ACC_1_METER;
    else
        return ODID_VER_ACC_UNKNOWN;
}


ODID_Speed_accuracy_t createEnumSpeedAccuracy(float Accuracy)
{
    if (Accuracy >= 10)
        return ODID_SPEED_ACC_UNKNOWN;
    else if (Accuracy >= 3)
        return ODID_SPEED_ACC_10_METERS_PER_SECOND;
    else if (Accuracy >= 1)
        return ODID_SPEED_ACC_3_METERS_PER_SECOND;
    else if (Accuracy >= 0.3f)
        return ODID_SPEED_ACC_1_METERS_PER_SECOND;
    else if (Accuracy > 0)
        return ODID_SPEED_ACC_0_3_METERS_PER_SECOND;
    else
        return ODID_SPEED_ACC_UNKNOWN;
}


ODID_Timestamp_accuracy_t createEnumTimestampAccuracy(float Accuracy)
{
    if (Accuracy > 1.5f)
        return ODID_TIME_ACC_UNKNOWN;
    else if (Accuracy > 1.4f)
        return ODID_TIME_ACC_1_5_SECOND;
    else if (Accuracy > 1.3f)
        return ODID_TIME_ACC_1_4_SECOND;
    else if (Accuracy > 1.2f)
        return ODID_TIME_ACC_1_3_SECOND;
    else if (Accuracy > 1.1f)
        return ODID_TIME_ACC_1_2_SECOND;
    else if (Accuracy > 1.0f)
        return ODID_TIME_ACC_1_1_SECOND;
    else if (Accuracy > 0.9f)
        return ODID_TIME_ACC_1_0_SECOND;
    else if (Accuracy > 0.8f)
        return ODID_TIME_ACC_0_9_SECOND;
    else if (Accuracy > 0.7f)
        return ODID_TIME_ACC_0_8_SECOND;
    else if (Accuracy > 0.6f)
        return ODID_TIME_ACC_0_7_SECOND;
    else if (Accuracy > 0.5f)
        return ODID_TIME_ACC_0_6_SECOND;
    else if (Accuracy > 0.4f)
        return ODID_TIME_ACC_0_5_SECOND;
    else if (Accuracy > 0.3f)
        return ODID_TIME_ACC_0_4_SECOND;
    else if (Accuracy > 0.2f)
        return ODID_TIME_ACC_0_3_SECOND;
    else if (Accuracy > 0.1f)
        return ODID_TIME_ACC_0_2_SECOND;
    else if (Accuracy > 0.0f)
        return ODID_TIME_ACC_0_1_SECOND;
    else
        return ODID_TIME_ACC_UNKNOWN;
}


float decodeHorizontalAccuracy(ODID_Horizontal_accuracy_t Accuracy)
{
    switch (Accuracy)
    {
    case ODID_HOR_ACC_UNKNOWN:
        return 18520;
    case ODID_HOR_ACC_10NM:
        return 18520;
    case ODID_HOR_ACC_4NM:
        return 7808;
    case ODID_HOR_ACC_2NM:
        return 3704;
    case ODID_HOR_ACC_1NM:
        return 1852;
    case ODID_HOR_ACC_0_5NM:
        return 926;
    case ODID_HOR_ACC_0_3NM:
        return 555.6f;
    case ODID_HOR_ACC_0_1NM:
        return 185.2f;
    case ODID_HOR_ACC_0_05NM:
        return 92.6f;
    case ODID_HOR_ACC_30_METER:
        return 30;
    case ODID_HOR_ACC_10_METER:
        return 10;
    case ODID_HOR_ACC_3_METER:
        return 3;
    case ODID_HOR_ACC_1_METER:
        return 1;
    default:
        return 18520;
    }
}


float decodeVerticalAccuracy(ODID_Vertical_accuracy_t Accuracy)
{
    switch (Accuracy)
    {
    case ODID_VER_ACC_UNKNOWN:
        return 150;
    case ODID_VER_ACC_150_METER:
        return 150;
    case ODID_VER_ACC_45_METER:
        return 45;
    case ODID_VER_ACC_25_METER:
        return 25;
    case ODID_VER_ACC_10_METER:
        return 10;
    case ODID_VER_ACC_3_METER:
        return 3;
    case ODID_VER_ACC_1_METER:
        return 1;
    default:
        return 150;
    }
}


float decodeSpeedAccuracy(ODID_Speed_accuracy_t Accuracy)
{
    switch (Accuracy)
    {
    case ODID_SPEED_ACC_UNKNOWN:
        return 10;
    case ODID_SPEED_ACC_10_METERS_PER_SECOND:
        return 10;
    case ODID_SPEED_ACC_3_METERS_PER_SECOND:
        return 3;
    case ODID_SPEED_ACC_1_METERS_PER_SECOND:
        return 1;
    case ODID_SPEED_ACC_0_3_METERS_PER_SECOND:
        return 0.3f;
    default:
        return 10;
    }
}


float decodeTimestampAccuracy(ODID_Timestamp_accuracy_t Accuracy)
{
    switch (Accuracy)
    {
    case ODID_TIME_ACC_UNKNOWN:
        return 0.0f;
    case ODID_TIME_ACC_0_1_SECOND:
        return 0.1f;
    case ODID_TIME_ACC_0_2_SECOND:
        return 0.2f;
    case ODID_TIME_ACC_0_3_SECOND:
        return 0.3f;
    case ODID_TIME_ACC_0_4_SECOND:
        return 0.4f;
    case ODID_TIME_ACC_0_5_SECOND:
        return 0.5f;
    case ODID_TIME_ACC_0_6_SECOND:
        return 0.6f;
    case ODID_TIME_ACC_0_7_SECOND:
        return 0.7f;
    case ODID_TIME_ACC_0_8_SECOND:
        return 0.8f;
    case ODID_TIME_ACC_0_9_SECOND:
        return 0.9f;
    case ODID_TIME_ACC_1_0_SECOND:
        return 1.0f;
    case ODID_TIME_ACC_1_1_SECOND:
        return 1.1f;
    case ODID_TIME_ACC_1_2_SECOND:
        return 1.2f;
    case ODID_TIME_ACC_1_3_SECOND:
        return 1.3f;
    case ODID_TIME_ACC_1_4_SECOND:
        return 1.4f;
    case ODID_TIME_ACC_1_5_SECOND:
        return 1.5f;
    default:
        return 0.0f;
    }
}

#ifndef ODID_DISABLE_PRINTF


void printByteArray(uint8_t *byteArray, uint16_t asize, int spaced)
{
    if (ENABLE_DEBUG) {
        int x;
        for (x=0;x<asize;x++) {
            printf("%02x", (unsigned int) byteArray[x]);
            if (spaced) {
                printf(" ");
            }
        }
        printf("\n");
    }
}


void printBasicID_data(ODID_BasicID_data *BasicID)
{

    char buf[ODID_ID_SIZE + 1] = { 0 };
    memcpy(buf, BasicID->UASID, ODID_ID_SIZE);

    const char ODID_BasicID_data_format[] =
        "UAType: %d\nIDType: %d\nUASID: %s\n";
    printf(ODID_BasicID_data_format, BasicID->IDType, BasicID->UAType, buf);
}


void printLocation_data(ODID_Location_data *Location)
{
    const char ODID_Location_data_format[] =
        "Status: %d\nDirection: %.1f\nSpeedHori: %.2f\nSpeedVert: "\
        "%.2f\nLat/Lon: %.7f, %.7f\nAlt: Baro, Geo, Height above %s: %.2f, "\
        "%.2f, %.2f\nHoriz, Vert, Baro, Speed, TS Accuracy: %.1f, %.1f, %.1f, "\
        "%.1f, %.1f\nTimeStamp: %.2f\n";
    printf(ODID_Location_data_format, Location->Status,
        (double) Location->Direction, (double) Location->SpeedHorizontal,
        (double) Location->SpeedVertical, Location->Latitude,
        Location->Longitude, Location->HeightType ? "Ground" : "TakeOff",
        (double) Location->AltitudeBaro, (double) Location->AltitudeGeo,
        (double) Location->Height,
        (double) decodeHorizontalAccuracy(Location->HorizAccuracy),
        (double) decodeVerticalAccuracy(Location->VertAccuracy),
        (double) decodeVerticalAccuracy(Location->BaroAccuracy),
        (double) decodeSpeedAccuracy(Location->SpeedAccuracy),
        (double) decodeTimestampAccuracy(Location->TSAccuracy),
        (double) Location->TimeStamp);
}


void printAuth_data(ODID_Auth_data *Auth)
{
    if (Auth->DataPage == 0) {
        const char ODID_Auth_data_format[] =
            "AuthType: %d\nDataPage: %d\nLastPageIndex: %d\nLength: %d\n"\
            "Timestamp: %u\nAuthData: ";
        printf(ODID_Auth_data_format, Auth->AuthType, Auth->DataPage,
               Auth->LastPageIndex, Auth->Length, Auth->Timestamp);
        for (int i = 0; i < ODID_AUTH_PAGE_ZERO_DATA_SIZE; i++)
            printf("0x%02X ", Auth->AuthData[i]);
    } else {
        const char ODID_Auth_data_format[] =
            "AuthType: %d\nDataPage: %d\nAuthData: ";
        printf(ODID_Auth_data_format, Auth->AuthType, Auth->DataPage);
        for (int i = 0; i < ODID_AUTH_PAGE_NONZERO_DATA_SIZE; i++)
            printf("0x%02X ", Auth->AuthData[i]);
    }
    printf("\n");
}


void printSelfID_data(ODID_SelfID_data *SelfID)
{

    char buf[ODID_STR_SIZE + 1] = { 0 };
    memcpy(buf, SelfID->Desc, ODID_STR_SIZE);

    const char ODID_SelfID_data_format[] = "DescType: %d\nDesc: %s\n";
    printf(ODID_SelfID_data_format, SelfID->DescType, buf);
}


void printSystem_data(ODID_System_data *System_data)
{
    const char ODID_System_data_format[] = "Operator Location Type: %d\n"
        "Classification Type: %d\nLat/Lon: %.7f, %.7f\n"
        "Area Count, Radius, Ceiling, Floor: %d, %d, %.2f, %.2f\n"
        "Category EU: %d, Class EU: %d, Altitude: %.2f, Timestamp: %u\n";
    printf(ODID_System_data_format, System_data->OperatorLocationType,
        System_data->ClassificationType,
        System_data->OperatorLatitude, System_data->OperatorLongitude,
        System_data->AreaCount, System_data->AreaRadius,
        (double) System_data->AreaCeiling, (double) System_data->AreaFloor,
        System_data->CategoryEU, System_data->ClassEU,
        (double) System_data->OperatorAltitudeGeo, System_data->Timestamp);
}


void printOperatorID_data(ODID_OperatorID_data *operatorID)
{

    char buf[ODID_ID_SIZE + 1] = { 0 };
    memcpy(buf, operatorID->OperatorId, ODID_ID_SIZE);

    const char ODID_OperatorID_data_format[] =
        "OperatorIdType: %d\nOperatorId: %s\n";
    printf(ODID_OperatorID_data_format, operatorID->OperatorIdType, buf);
}

#endif // ODID_DISABLE_PRINTF

