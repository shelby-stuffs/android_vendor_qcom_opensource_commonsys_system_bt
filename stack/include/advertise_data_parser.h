/******************************************************************************
 *
 *  Copyright (C) 2017 The Android Open Source Project
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * ​​​​​Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ******************************************************************************/

#pragma once

#include <array>
#include <vector>
#include <map>

#define BTM_BLE_AD_TYPE_ED 0x31

// Scan Response data from Traxxas
static constexpr std::array<uint8_t, 18> trx_quirk{
    {0x14, 0x09, 0x54, 0xFF, 0xFF, 0x20, 0x42, 0x4C, 0x45, 0x05, 0x12, 0xFF,
     0x00, 0xE8, 0x03, 0x02, 0x0A, 0x00}};

class AdvertiseDataParser {
  // Return true if the packet is malformed, but should be considered valid for
  // compatibility with already existing devices
  static bool MalformedPacketQuirk(const std::vector<uint8_t>& ad,
                                   size_t position) {
    auto data_start = ad.begin() + position;

    // Traxxas - bad name length
    if ((ad.size() - position) >= 18 &&
        std::equal(data_start, data_start + 3, trx_quirk.begin()) &&
        std::equal(data_start + 5, data_start + 11, trx_quirk.begin() + 5) &&
        std::equal(data_start + 12, data_start + 18, trx_quirk.begin() + 12)) {
      return true;
    }

    return false;
  }

 public:
  static void RemoveTrailingZeros(std::vector<uint8_t>& ad) {
    size_t position = 0;

    size_t ad_len = ad.size();
    while (position < ad_len) {
      uint8_t len = ad[position];

      // A field length of 0 would be invalid as it should at least contain the
      // EIR field type. However, some existing devices send zero padding at the
      // end of advertisement. If this is the case, cut the zero padding from
      // end of the packet. Otherwise i.e. gluing scan response to advertise
      // data will result in data with zero padding in the middle.
      if (len == 0) {
        ad.erase(ad.begin() + position, ad.end());
        return;
      }

      if (position + len >= ad_len) {
        return;
      }

      position += len + 1;
    }
  }

  /**
   * Return true if this |ad| represent properly formatted advertising data.
   */
  static bool IsValid(const std::vector<uint8_t>& ad) {
    size_t position = 0;

    size_t ad_len = ad.size();
    while (position < ad_len) {
      uint8_t len = ad[position];

      // A field length of 0 would be invalid as it should at least contain the
      // EIR field type. However, some existing devices send zero padding at the
      // end of advertisement. If this is the case, treat the packet as valid.
      if (len == 0) {
        for (size_t i = position + 1; i < ad_len; i++) {
          if (ad[i] != 0) return false;
        }
        return true;
      }

      // If the length of the current field would exceed the total data length,
      // then the data is badly formatted.
      if (position + len >= ad_len) {
        if (MalformedPacketQuirk(ad, position)) return true;

        return false;
      }

      position += len + 1;
    }

    return true;
  }

  /**
   * This function returns a pointer inside the |ad| array of length |ad_len|
   * where a field of |type| is located, together with its length in |p_length|
   */
  static const uint8_t* GetFieldByType(const uint8_t* ad, size_t ad_len,
                                       uint8_t type, uint8_t* p_length) {
    size_t position = 0;

    while (position < ad_len) {
      uint8_t len = ad[position];

      if (len == 0) break;
      if (position + len >= ad_len) break;

      uint8_t adv_type = ad[position + 1];

      if (adv_type == type) {
        /* length doesn't include itself */
        *p_length = len - 1; /* minus the length of type */
        return ad + position + 2;
      }

      position += len + 1; /* skip the length of data */
    }

    *p_length = 0;
    return NULL;
  }

  /**
   * This function returns a pointer inside the |adv| where a field of |type| is
   * located, together with it' length in |p_length|
   */
  static const uint8_t* GetFieldByType(std::vector<uint8_t> const& ad,
                                       uint8_t type, uint8_t* p_length) {
    return GetFieldByType(ad.data(), ad.size(), type, p_length);
  }

  /**
  * This function returns a map<starting position of Enc AD data part in original
  * Adv data, Length of Enc AD data part (including L, T(of Enc AD type) in |p_length|>
  *
  */
  static std::map<int, int> GetEncAdvFieldsInfo(const uint8_t* ad, size_t ad_len) {
    size_t position = 0;
    std::map<int, int> enc_adv_map;
    int enc_data_part_length = 0;

    while (position < ad_len) {
      uint8_t len = ad[position];

      if (len == 0) break;
      if (position + len >= ad_len) break;

      uint8_t adv_type = ad[position + 1];

      if (adv_type == BTM_BLE_AD_TYPE_ED) {
        enc_data_part_length = len + 1; /* Length(1 byte) + len */
        enc_adv_map.insert(std::pair<int, int>((int)position, enc_data_part_length));
      }

      position += len + 1; /* skip the length of data */
    }

    return enc_adv_map;
  }
};
