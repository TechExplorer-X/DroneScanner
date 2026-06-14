/*
 * Open Drone ID C Library - WiFi frame receive/process
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(ARDUINO_ARCH_ESP32)
#include <Arduino.h>
int clock_gettime(clockid_t, struct timespec *);
#else
#include <string.h>
#include <stddef.h>
#include <stdio.h>
#endif

#include <errno.h>
#include <time.h>

#include "opendroneid.h"
#include "odid_wifi.h"

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#if defined(IDF_VER)
#include <endian.h>
#define cpu_to_be16(x)  (bswap16(x))
#define cpu_to_be32(x)  (bswap32(x))
#else
#include <byteswap.h>
#define cpu_to_be16(x)  (bswap_16(x))
#define cpu_to_be32(x)  (bswap_32(x))
#endif
#define cpu_to_le16(x)  (x)
#define cpu_to_le64(x)  (x)
#else
#define cpu_to_be16(x)      (x)
#define cpu_to_be32(x)      (x)
#define cpu_to_le16(x)      (bswap_16(x))
#define cpu_to_le64(x)      (bswap_64(x))
#endif

#define IEEE80211_FCTL_FTYPE          0x000c
#define IEEE80211_FCTL_STYPE          0x00f0

#define IEEE80211_FTYPE_MGMT            0x0000
#define IEEE80211_STYPE_ACTION          0x00D0
#define IEEE80211_STYPE_BEACON          0x0080

/* frame markers */
#define PKT_MARKER_0     { 0x51, 0x6F, 0x9A, 0x01, 0x00, 0x00 }
#define OUI_MARKER        { 0x50, 0x6F, 0x9A }
#define SVC_MARKER        { 0x88, 0x69, 0x19, 0x9D, 0x92, 0x09 }
#define FRAME_TYPE_B      0x80
#define ELEM_TAG          0xdd
#define VND_MARK_A        0x90, 0x3a, 0xe6
#define VND_MARK_B        0xfa, 0x0b, 0xbc
#define HDR_SKIP          36


int rf_data_decode(ODID_UAS_Data *UAS_Data, uint8_t *pack, size_t buflen)
{
    ODID_MessagePack_encoded *penc = (ODID_MessagePack_encoded *) pack;
    size_t size = sizeof(*penc) - ODID_MESSAGE_SIZE * (ODID_PACK_MAX_MESSAGES - penc->MsgPackSize);
    if (size > buflen)
        return -ENOMEM;

    odid_initUasData(UAS_Data);

    if (decodeMessagePack(UAS_Data, penc) != ODID_SUCCESS)
        return -1;

    return (int) size;
}

int rf_recv_pkt_a(ODID_UAS_Data *UAS_Data,
                                                    char *mac, uint8_t *buf, size_t buf_size)
{
    struct ieee80211_mgmt *mgmt;
    struct nan_service_discovery *nsd;
    struct nan_service_descriptor_attribute *nsda;
    struct nan_service_descriptor_extension_attribute *nsdea;
    struct svc_info *si;
    uint8_t target_addr[6] = PKT_MARKER_0;
    uint8_t wifi_alliance_oui[3] = OUI_MARKER;
    uint8_t service_id[6] = SVC_MARKER;
    int ret;
    size_t len = 0;

    if (len + sizeof(*mgmt) > buf_size)
        return -EINVAL;
    mgmt = (struct ieee80211_mgmt *)(buf + len);
    if ((mgmt->frame_control & cpu_to_le16(IEEE80211_FCTL_FTYPE | IEEE80211_FCTL_STYPE)) !=
        cpu_to_le16(IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_ACTION))
        return -EINVAL;
    if (memcmp(mgmt->da, target_addr, sizeof(mgmt->da)) != 0)
        return -EINVAL;
    memcpy(mac, mgmt->sa, sizeof(mgmt->sa));

    len += sizeof(*mgmt);

    if (len + sizeof(*nsd) > buf_size)
        return -EINVAL;
    nsd = (struct nan_service_discovery *)(buf + len);
    if (nsd->category != 0x04)
        return -EINVAL;
    if (nsd->action_code != 0x09)
        return -EINVAL;
    if (memcmp(nsd->oui, wifi_alliance_oui, sizeof(wifi_alliance_oui)) != 0)
        return -EINVAL;
    if (nsd->oui_type != 0x13)
        return -EINVAL;
    len += sizeof(*nsd);

    if (len + sizeof(*nsda) > buf_size)
        return -EINVAL;
    nsda = (struct nan_service_descriptor_attribute *)(buf + len);
    if (nsda->header.attribute_id != 0x3)
        return -EINVAL;
    if (memcmp(nsda->service_id, service_id, sizeof(service_id)) != 0)
        return -EINVAL;
    if (nsda->instance_id != 0x01)
        return -EINVAL;
    if (nsda->service_control != 0x10)
        return -EINVAL;
    len += sizeof(*nsda);

    si = (struct svc_info *)(buf + len);
    ret = rf_data_decode(UAS_Data, buf + len + sizeof(*si), buf_size - len - sizeof(*nsdea));
    if (ret < 0)
        return -EINVAL;
    if (nsda->service_info_length != (sizeof(*si) + ret))
        return -EINVAL;
    if (nsda->header.length != (cpu_to_le16(sizeof(*nsda) - sizeof(struct nan_attribute_header) + nsda->service_info_length)))
        return -EINVAL;
    len += sizeof(*si) + ret;

    if (len + sizeof(*nsdea) > buf_size)
        return -ENOMEM;
    nsdea = (struct nan_service_descriptor_extension_attribute *)(buf + len);
    if (nsdea->header.attribute_id != 0xE)
        return -EINVAL;
    if (nsdea->header.length != cpu_to_le16(0x0004))
        return -EINVAL;
    if (nsdea->instance_id != 0x01)
        return -EINVAL;
    if (nsdea->control != cpu_to_le16(0x0200))
        return -EINVAL;

    return 0;
}
