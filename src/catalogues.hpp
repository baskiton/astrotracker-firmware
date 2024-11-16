#include <FS.h>
#include <LittleFS.h>

#include "coordinates.hpp"
#include "settings.h"

typedef struct __attribute__((packed)) {
    float ra;   // sec
    float dec;  // sec
} cat_t;

class Catalogues {
public:
    bool init() {
        hr_cat_f = LittleFS.open(F_STR(HR_CAT_NAME), "r");
        starnames_hr_f = LittleFS.open(F_STR(STARNAMES_HR_NAME), "r");
        messier_cat_f = LittleFS.open(F_STR(MESSIER_CAT_NAME), "r");
        m_names_f = LittleFS.open(F_STR(M_NAMES_NAME), "r");
        ngc_cat_f = LittleFS.open(F_STR(NGC_CAT_NAME), "r");
        ic_cat_f = LittleFS.open(F_STR(IC_CAT_NAME), "r");
        return hr_cat_f && starnames_hr_f && messier_cat_f && m_names_f && ngc_cat_f && ic_cat_f;
    }

    void by_hr(uint16_t num, uint32_t ts, cat_t &entry) {
        hr_cat_f.seek(sizeof(cat_t) * (num - 1));
        hr_cat_f.read((uint8_t *)&entry, sizeof(cat_t));
        eq_coord_t to{ .epoch = UTS_TO_JDN(ts), };
        precession_correction({ entry.ra, entry.dec, J2000 }, to);
        entry.ra = to.ra;
        entry.dec = to.dec;
    }

    uint16_t by_satname_idx(uint16_t idx, uint32_t ts, cat_t &entry) {
        starnames_hr_f.seek(sizeof(idx) * idx);
        uint16_t num;
        starnames_hr_f.read((uint8_t *)&num, sizeof(num));
        by_hr(num, ts, entry);
        return num;
    }

    void by_messier(uint16_t num, uint32_t ts, cat_t &entry) {
        messier_cat_f.seek(sizeof(cat_t) * (num - 1));
        messier_cat_f.read((uint8_t *)&entry, sizeof(cat_t));
        eq_coord_t to{ .epoch = UTS_TO_JDN(ts), };
        precession_correction({ entry.ra, entry.dec, J2000 }, to);
        entry.ra = to.ra;
        entry.dec = to.dec;
    }

    uint16_t by_m_name_idx(uint16_t idx, uint32_t ts, cat_t &entry) {
        m_names_f.seek(sizeof(idx) * idx);
        uint16_t num;
        m_names_f.read((uint8_t *)&num, sizeof(num));
        by_messier(num, ts, entry);
        return num;
    }

    void by_ngc(uint16_t num, uint32_t ts, cat_t &entry) {
        ngc_cat_f.seek(sizeof(cat_t) * (num - 1));
        ngc_cat_f.read((uint8_t *)&entry, sizeof(cat_t));
        eq_coord_t to{ .epoch = UTS_TO_JDN(ts), };
        precession_correction({ entry.ra, entry.dec, J2000 }, to);
        entry.ra = to.ra;
        entry.dec = to.dec;
    }

    void by_ic(uint16_t num, uint32_t ts, cat_t &entry) {
        ic_cat_f.seek(sizeof(cat_t) * (num - 1));
        ic_cat_f.read((uint8_t *)&entry, sizeof(cat_t));
        eq_coord_t to{ .epoch = UTS_TO_JDN(ts), };
        precession_correction({ entry.ra, entry.dec, J2000 }, to);
        entry.ra = to.ra;
        entry.dec = to.dec;
    }

    int16_t by_ngcic_name_idx(uint16_t idx, uint32_t ts, cat_t &entry) {
        m_names_f.seek(sizeof(idx) * idx);
        int16_t num;
        m_names_f.read((uint8_t *)&num, sizeof(num));
        if (num  > 0)
            by_ngc(num, ts, entry);
        else
            by_ic(abs(num), ts, entry);
        return num;
    }

private:
    File hr_cat_f;
    File starnames_hr_f;
    File messier_cat_f;
    File m_names_f;
    File ngc_cat_f;
    File ic_cat_f;
};
