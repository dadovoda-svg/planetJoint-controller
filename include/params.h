#pragma once

#include <Arduino.h>
#include <Preferences.h>

class PersistentParams {
public:
    static constexpr uint8_t MAX_KEY_LEN = 6;
    static constexpr uint8_t MAX_PARAMS  = 32;

    enum class LoadResult {
        Ok,
        NotFound,
        InvalidData,
        CrcError,
        TooManyParams
    };

    PersistentParams(const char* nvsNamespace = "params",
                     const char* nvsKey = "blob")
        : _namespace(nvsNamespace), _nvsKey(nvsKey) {}

    bool begin() {
        return _prefs.begin(_namespace, false);
    }

    void end() {
        _prefs.end();
    }

    bool initKey(const char* key, float defaultValue) {
        if (!isValidKey(key)) {
            return false;
        }

        int index = findIndex(key);

        if (index >= 0) {
            return true;
        }

        if (_count >= MAX_PARAMS) {
            return false;
        }

        strncpy(_items[_count].key, key, MAX_KEY_LEN);
        _items[_count].key[MAX_KEY_LEN] = '\0';
        _items[_count].value = defaultValue;
        _count++;

        return true;
    }

    bool set(const char* key, float value) {
        if (!isValidKey(key)) {
            return false;
        }

        int index = findIndex(key);

        if (index < 0) {
            return false;
        }

        _items[index].value = value;
        return true;
    }

    bool get(const char* key, float& value) const {
        if (!isValidKey(key)) {
            return false;
        }

        int index = findIndex(key);

        if (index < 0) {
            return false;
        }

        value = _items[index].value;
        return true;
    }

    bool exists(const char* key) const {
        return findIndex(key) >= 0;
    }

    uint8_t count() const {
        return _count;
    }

    bool getByIndex(uint8_t index, const char*& key, float& value) const {
        if (index >= _count) {
            return false;
        }

        key = _items[index].key;
        value = _items[index].value;

        return true;
    }

    bool save() {
        StorageImage image {};
        image.magic = MAGIC;
        image.version = VERSION;
        image.count = _count;

        for (uint8_t i = 0; i < _count; i++) {
            image.items[i] = _items[i];
        }

        image.crc32 = 0;
        image.crc32 = computeCrc32(
            reinterpret_cast<const uint8_t*>(&image),
            sizeof(StorageImage)
        );

        size_t written = _prefs.putBytes(_nvsKey, &image, sizeof(StorageImage));

        return written == sizeof(StorageImage);
    }

    LoadResult loadDetailed() {
        size_t size = _prefs.getBytesLength(_nvsKey);

        if (size == 0) {
            return LoadResult::NotFound;
        }

        if (size != sizeof(StorageImage)) {
            return LoadResult::InvalidData;
        }

        StorageImage image {};
        size_t read = _prefs.getBytes(_nvsKey, &image, sizeof(StorageImage));

        if (read != sizeof(StorageImage)) {
            return LoadResult::InvalidData;
        }

        if (image.magic != MAGIC || image.version != VERSION) {
            return LoadResult::InvalidData;
        }

        if (image.count > MAX_PARAMS) {
            return LoadResult::TooManyParams;
        }

        uint32_t receivedCrc = image.crc32;
        image.crc32 = 0;

        uint32_t calculatedCrc = computeCrc32(
            reinterpret_cast<const uint8_t*>(&image),
            sizeof(StorageImage)
        );

        if (receivedCrc != calculatedCrc) {
            return LoadResult::CrcError;
        }

        for (uint8_t i = 0; i < image.count; i++) {
            if (!isValidKey(image.items[i].key)) {
                return LoadResult::InvalidData;
            }
        }

        /*
        * Merge implicita:
        * - aggiorna solo le chiavi già dichiarate con initKey()
        * - ignora le chiavi obsolete presenti in NVS
        * - mantiene i default delle chiavi nuove
        */
        for (uint8_t i = 0; i < image.count; i++) {
            int ramIndex = findIndex(image.items[i].key);

            if (ramIndex >= 0) {
                _items[ramIndex].value = image.items[i].value;
            }
        }

        return LoadResult::Ok;
    }
    
    bool load() {
        return loadDetailed() == LoadResult::Ok;
    }

private:
    struct ParamItem {
        char key[MAX_KEY_LEN + 1];
        float value;
    };

    struct StorageImage {
        uint32_t magic;
        uint16_t version;
        uint16_t count;
        ParamItem items[MAX_PARAMS];
        uint32_t crc32;
    };

    static constexpr uint32_t MAGIC = 0x504A5053; // "PJPS" circa: PlanetJoint Params Storage
    static constexpr uint16_t VERSION = 1;

    Preferences _prefs;
    const char* _namespace;
    const char* _nvsKey;

    ParamItem _items[MAX_PARAMS] {};
    uint8_t _count = 0;

    bool isValidKey(const char* key) const {
        if (key == nullptr) {
            return false;
        }

        size_t len = strlen(key);

        if (len == 0 || len > MAX_KEY_LEN) {
            return false;
        }

        for (size_t i = 0; i < len; i++) {
            if (!isalnum(static_cast<unsigned char>(key[i]))) {
                return false;
            }
        }

        return true;
    }

    int findIndex(const char* key) const {
        for (uint8_t i = 0; i < _count; i++) {
            if (strcmp(_items[i].key, key) == 0) {
                return i;
            }
        }

        return -1;
    }

    static uint32_t computeCrc32(const uint8_t* data, size_t length) {
        uint32_t crc = 0xFFFFFFFF;

        for (size_t i = 0; i < length; i++) {
            crc ^= data[i];

            for (uint8_t bit = 0; bit < 8; bit++) {
                if (crc & 1) {
                    crc = (crc >> 1) ^ 0xEDB88320;
                } else {
                    crc >>= 1;
                }
            }
        }

        return ~crc;
    }
};